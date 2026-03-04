#!/usr/bin/env python3
"""
ROS2-to-Web Bridge Node
=======================
Bridges the multi-robot SLAM simulation to the SLAM Visualization Platform
web client (http://localhost:8000).

Data flow
---------
  /merged_map  (nav_msgs/OccupancyGrid)  → GET /api/map   (3-D point cloud)
  /fused_map   (nav_msgs/OccupancyGrid)  → GET /api/map   (fallback source)
  /robotN/odom (nav_msgs/Odometry)       → WebSocket /ws  (MULTI_ROBOT_POSES)

The node runs a FastAPI/uvicorn HTTP server in a daemon thread and exposes
every REST endpoint the Next.js frontend already calls. No changes to the
simulation or the frontend transport layer are required.

Dependencies (pip)
------------------
  pip install fastapi uvicorn[standard]
"""

import asyncio
import json
import math
import threading
from datetime import datetime, timezone
from typing import Dict, List, Optional, Set

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import uvicorn


# ── Robot metadata ─────────────────────────────────────────────────────────────

ROBOT_NAMES: List[str] = ["robot1", "robot2", "robot3", "robot4"]

# Hex colours shown in the Three.js canvas for each robot
ROBOT_COLORS: Dict[str, str] = {
    "robot1": "orange",
    "robot2": "#22d3ee",   # cyan-400
    "robot3": "#4ade80",   # green-400
    "robot4": "#f472b6",   # pink-400
}


# Shared state (written by ROS thread, read by FastAPI)

class _BridgeState:
    def __init__(self) -> None:
        self.map_points: List[dict] = []
        self.map_version: int = 0
        self.robot_poses: Dict[str, dict] = {}
        self.ws_clients: Set[WebSocket] = set()
        self.lock = threading.Lock()
        self.event_loop: Optional[asyncio.AbstractEventLoop] = None
        self.started_at: str = datetime.now(timezone.utc).isoformat()


_state = _BridgeState()


# FastAPI application

app = FastAPI(title="SLAM Visualization Bridge", version="1.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# helpers

async def _broadcast(message: str) -> None:
    """Forward *message* to every connected WebSocket client."""
    dead: Set[WebSocket] = set()
    with _state.lock:
        clients = list(_state.ws_clients)
    for ws in clients:
        try:
            await ws.send_text(message)
        except Exception:
            dead.add(ws)
    if dead:
        with _state.lock:
            _state.ws_clients -= dead


def _schedule_broadcast(message: str) -> None:
    """Thread-safe helper: schedule a broadcast from a sync ROS callback."""
    loop = _state.event_loop
    if loop and loop.is_running():
        asyncio.run_coroutine_threadsafe(_broadcast(message), loop)


# REST endpoints

@app.get("/api/map")
async def get_map(version: int = 0):
    """
    Returns the current merged-map point cloud (or 'no_update' when unchanged).
    """
    with _state.lock:
        current_version = _state.map_version
        if current_version == version:
            return {"status": "no_update", "version": version, "points": []}
        return {
            "status": "ok",
            "version": current_version,
            "points": _state.map_points,
        }


@app.get("/api/system/status")
async def get_status():
    """Polled by SideBar.tsx every 2 s."""
    with _state.lock:
        active = [rid for rid, p in _state.robot_poses.items() if p]
    return {
        # Keys expected by SideBar.tsx (in SLAM Visualization Platform)
        "simulator_status": "CONNECTED",
        "last_heartbeat": datetime.now(timezone.utc).isoformat(),
        # Extra info for diagnostics
        "active_robots": active,
        "robot_count": len(active),
        "map_points": len(_state.map_points),
        "map_version": _state.map_version,
        "bridge_started_at": _state.started_at,
    }


@app.post("/api/command/waypoint")
async def set_waypoint(body: dict):
    """
    Called by MapCanvas.tsx when the user clicks to deploy a robot.
    TODO: publish geometry_msgs/PoseStamped to /<robot_id>/move_base_simple/goal
    """
    robot_id = body.get("robot_id", "robot1")
    x = body.get("x", 0.0)
    z = body.get("z", 0.0)      # Three.js z == ROS y
    return {
        "status": "ok",
        "message": f"Waypoint ({x:.2f}, {z:.2f}) queued for {robot_id}",
    }


@app.post("/api/command/emergency_stop")
async def emergency_stop():
    """
    TODO: publish std_msgs/Empty to /emergency_stop for every robot.
    """
    return {"status": "ok", "message": "Emergency stop triggered"}


@app.get("/api/annotations")
async def get_annotations():
    return JSONResponse(content=[])


@app.post("/api/annotations")
async def create_annotation(body: dict):
    return JSONResponse(
        content={"id": f"bridge-{id(body)}", **body},
        status_code=201,
    )


@app.post("/api/map/snapshot")
async def save_snapshot():
    return {"status": "ok", "message": "Snapshot saved (bridge stub)"}


@app.post("/api/map/load")
async def load_snapshot():
    return {"status": "ok", "message": "Snapshot loaded (bridge stub)"}


@app.delete("/api/map/clear")
async def clear_map():
    with _state.lock:
        _state.map_points = []
        _state.map_version += 1
    return {"status": "ok"}


@app.get("/api/analytics/collisions")
async def get_collisions():
    return JSONResponse(content=[])


# WebSocket endpoint

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    with _state.lock:
        _state.ws_clients.add(ws)
        # Send current poses immediately so the client doesn't wait
        initial_robots = list(_state.robot_poses.values())

    if initial_robots:
        await ws.send_text(
            json.dumps({"type": "MULTI_ROBOT_POSES", "robots": initial_robots})
        )

    try:
        while True:
            # We don't need to receive anything; keep the connection alive.
            await asyncio.sleep(30)
    except (WebSocketDisconnect, Exception):
        pass
    finally:
        with _state.lock:
            _state.ws_clients.discard(ws)


# ROS2 Node

class WebBridgeNode(Node):
    """
    Subscribes to SLAM topics and feeds the FastAPI state object.
    Both /merged_map (map_merger_node) and /fused_map (map_fusion_node)
    are accepted so either launch configuration works.
    """

    def __init__(self) -> None:
        super().__init__("web_bridge")

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
            "Web Bridge Node started – SLAM data available at http://0.0.0.0:8000"
        )

    # ── map callback ─────────────────────────────────────────────────────────

    def _map_cb(self, msg: OccupancyGrid) -> None:
        """
        Convert a 2-D OccupancyGrid into the list[{x,y,z,confidence}] format
        that SlamMap.tsx renders as a Three.js point cloud.
        """
        width      = msg.info.width
        height     = msg.info.height
        resolution = msg.info.resolution
        orig_x     = msg.info.origin.position.x
        orig_y     = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        points: List[dict] = []
        # Sample every 2nd cell: enough detail, half the payload
        step = 2

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

        with _state.lock:
            _state.map_points = points
            _state.map_version += 1
            new_version = _state.map_version

        self.get_logger().info(
            f"Map updated: {len(points)} points (v{new_version})",
            throttle_duration_sec=5.0,
        )

    # ── odometry callback ────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry, robot_name: str) -> None:
        """
        Extract pose from odometry and push it to every connected WebSocket
        client as both MULTI_ROBOT_POSES (multi-robot) and a legacy
        ROBOT_POSE message (backwards-compatible with the current frontend).
        """
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        qz  = msg.pose.pose.orientation.z
        qw  = msg.pose.pose.orientation.w
        yaw = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))

        pose = {
            "id":     robot_name,
            "color":  ROBOT_COLORS.get(robot_name, "orange"),
            "x":      round(x, 3),
            "z":      round(y, 3),   # ROS y → Three.js z
            "angle":  round(yaw, 4),
            "status": "MAPPING",
        }

        with _state.lock:
            _state.robot_poses[robot_name] = pose
            all_poses = list(_state.robot_poses.values())

        # Multi-robot message (new)
        msg_multi = json.dumps({"type": "MULTI_ROBOT_POSES", "robots": all_poses})

        # Single-robot legacy message using robot1 (or first available)
        primary = next(
            (p for p in all_poses if p["id"] == "robot1"),
            all_poses[0] if all_poses else None,
        )
        msg_single: Optional[str] = None
        if primary:
            msg_single = json.dumps(
                {
                    "type":   "ROBOT_POSE",
                    "x":      primary["x"],
                    "z":      primary["z"],
                    "angle":  primary["angle"],
                    "status": primary["status"],
                }
            )

        for m in filter(None, [msg_multi, msg_single]):
            _schedule_broadcast(m)


# Server bootstrap

def _run_server() -> None:
    """Run FastAPI/uvicorn in a daemon thread with its own event loop."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    _state.event_loop = loop

    config = uvicorn.Config(
        app,
        host="0.0.0.0",
        port=8000,
        loop="none",        # we manage the loop ourselves
        log_level="warning",
    )
    server = uvicorn.Server(config)
    loop.run_until_complete(server.serve())


def main(args=None) -> None:
    rclpy.init(args=args)

    server_thread = threading.Thread(target=_run_server, daemon=True, name="uvicorn")
    server_thread.start()

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
