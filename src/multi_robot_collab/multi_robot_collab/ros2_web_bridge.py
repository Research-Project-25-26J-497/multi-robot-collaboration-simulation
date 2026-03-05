#!/usr/bin/env python3
"""
ROS2-to-Backend Bridge Node
============================
Bridges the multi-robot SLAM simulation to the SLAM Visualization Platform
backend API (http://localhost:8000).

Data flow
---------
  /merged_map  (nav_msgs/OccupancyGrid)  → POST /api/map/batch
  /fused_map   (nav_msgs/OccupancyGrid)  → POST /api/map/batch   (fallback)
  /robotN/odom (nav_msgs/Odometry)       → POST /api/robot/telemetry

The node forwards real SLAM data to the backend so heatmaps, annotations,
and map snapshots reflect actual robot activity instead of the sample
simulate_slam.py data.

Dependencies (pip)
------------------
  pip install requests numpy
"""

import math
import os
import threading
import time
from typing import Dict, List

import numpy as np
import requests
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node


# ── Robot metadata ─────────────────────────────────────────────────────────────

ROBOT_NAMES: List[str] = ["robot1", "robot2", "robot3", "robot4"]

# Hex colours matching the Three.js canvas and backend ROBOT_COLORS
ROBOT_COLORS: Dict[str, str] = {
    "robot1": "orange",
    "robot2": "#22d3ee",   # cyan-400
    "robot3": "#4ade80",   # green-400
    "robot4": "#f472b6",   # pink-400
}

# ── Bridge configuration ────────────────────────────────────────────────────────

# Minimum seconds between map batch POSTs to avoid overloading the backend
MAP_THROTTLE_SEC = 2.0


def _resolve_backend_url() -> str:
    """
    Resolve the backend URL with the following priority:
      1. MANTIS_BACKEND_URL environment variable (always wins)
      2. Auto-detected Windows host IP when running inside WSL2
      3. http://localhost:8000 (same-machine fallback)

    When the backend runs on Windows and the simulation runs in WSL2,
    'localhost' inside WSL2 does NOT reach Windows.  The Windows host is
    reachable via the 'nameserver' entry in /etc/resolv.conf (WSL2 injects
    the host IP there).
    """
    env_url = os.environ.get("MANTIS_BACKEND_URL", "").strip()
    if env_url:
        return env_url

    # Detect WSL2: /proc/version contains 'microsoft' on WSL kernels
    try:
        with open("/proc/version", "r") as fh:
            if "microsoft" in fh.read().lower():
                import subprocess
                result = subprocess.run(
                    ["ip", "route", "show"],
                    capture_output=True, text=True, timeout=3
                )
                for line in result.stdout.splitlines():
                    if line.startswith("default"):
                        parts = line.split()
                        via_idx = parts.index("via") if "via" in parts else -1
                        if via_idx != -1 and via_idx + 1 < len(parts):
                            host_ip = parts[via_idx + 1]
                            return f"http://{host_ip}:8000"
    except (OSError, IndexError, ValueError, Exception):
        pass

    return "http://localhost:8000"


BACKEND_URL = _resolve_backend_url()


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class WebBridgeNode(Node):
    """
    Subscribes to SLAM topics and POSTs data to the backend API.
    Both /merged_map (map_merger_node) and /fused_map (map_fusion_node)
    are accepted so either launch configuration works.
    """

    def __init__(self) -> None:
        super().__init__("web_bridge")
        self._last_map_post: float = 0.0
        self._lock = threading.Lock()

        # Accept data from both merger implementations
        self.create_subscription(OccupancyGrid, "/merged_map", self._map_cb, 10)
        self.create_subscription(OccupancyGrid, "/fused_map",  self._map_cb, 10)

        # Per-robot odometry
        for robot_name in ROBOT_NAMES:
            self.create_subscription(
                Odometry,
                f"/{robot_name}/odom",
                lambda msg, rn=robot_name: self._odom_cb(msg, rn),
                10,
            )

        self.get_logger().info(
            f"Web Bridge Node started – forwarding SLAM data to {BACKEND_URL}"
        )
        self._check_backend_reachable()

    # ── startup check ────────────────────────────────────────────────────────

    def _check_backend_reachable(self) -> None:
        """Log whether the backend is reachable at startup."""
        try:
            resp = requests.get(f"{BACKEND_URL}/", timeout=3.0)
            self.get_logger().info(
                f"Backend reachable at {BACKEND_URL}  (status {resp.status_code})"
            )
        except requests.exceptions.RequestException as exc:
            self.get_logger().error(
                f"[BRIDGE] Cannot reach backend at {BACKEND_URL}: {exc}\n"
                f"  If the backend runs on Windows and this node is in WSL2,\n"
                f"  set the MANTIS_BACKEND_URL env var to the Windows host IP:\n"
                f"    export MANTIS_BACKEND_URL=http://<windows-host-ip>:8000\n"
                f"  (find it with: ip route show | grep default | awk '{{print $3}}')"
            )

    # ── map callback ─────────────────────────────────────────────────────────

    def _map_cb(self, msg: OccupancyGrid) -> None:
        """
        Convert OccupancyGrid → point cloud and POST to /api/map/batch.
        Throttled to MAP_THROTTLE_SEC to avoid overloading the backend.
        """
        now = time.monotonic()
        with self._lock:
            if now - self._last_map_post < MAP_THROTTLE_SEC:
                return
            self._last_map_post = now

        width      = msg.info.width
        height     = msg.info.height
        resolution = msg.info.resolution
        orig_x     = msg.info.origin.position.x
        orig_y     = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        points: List[dict] = []
        step = 2  # sample every 2nd cell for performance

        for ry in range(0, height, step):
            for rx in range(0, width, step):
                value = int(data[ry, rx])
                if value < 50:      # skip unknown (-1) and free space (0–49)
                    continue

                ros_x = orig_x + (rx + 0.5) * resolution
                ros_y = orig_y + (ry + 0.5) * resolution
                confidence = round(min(1.0, value / 100.0), 2)

                # Three stacked points give each wall cell height in 3-D
                for lvl in (0.0, 0.5, 1.0):
                    points.append(
                        {
                            "x": round(ros_x, 3),
                            "y": lvl,
                            "z": round(ros_y, 3),
                            "confidence": confidence,
                        }
                    )

        if not points:
            return

        try:
            requests.post(
                f"{BACKEND_URL}/api/map/batch",
                json=points,
                timeout=2.0,
            )
            self.get_logger().info(
                f"Map forwarded: {len(points)} points",
                throttle_duration_sec=5.0,
            )
        except requests.exceptions.RequestException as exc:
            self.get_logger().warn(
                f"Map POST failed: {exc}",
                throttle_duration_sec=5.0,
            )

    # ── odometry callback ────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry, robot_name: str) -> None:
        """
        Extract pose from odometry and POST to /api/robot/telemetry.
        The backend stores all robot poses and broadcasts MULTI_ROBOT_POSES
        to every connected WebSocket client.
        """
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        qz  = msg.pose.pose.orientation.z
        qw  = msg.pose.pose.orientation.w
        yaw = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))

        payload = {
            "robot_id": robot_name,
            "color":    ROBOT_COLORS.get(robot_name, "orange"),
            "x":        round(x, 3),
            "z":        round(y, 3),   # ROS y → Three.js z
            "angle":    round(yaw, 4),
            "status":   "MAPPING",
        }

        try:
            requests.post(
                f"{BACKEND_URL}/api/robot/telemetry",
                json=payload,
                timeout=0.2,
            )
        except requests.exceptions.RequestException:
            pass  # backend not reachable yet, skip silently


# ── Entry point ─────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
