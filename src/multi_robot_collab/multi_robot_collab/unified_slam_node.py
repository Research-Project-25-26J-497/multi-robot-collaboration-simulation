#!/usr/bin/env python3
"""
Unified Multi-Robot SLAM Node
Single SLAM instance that processes scans from all robots for perfect alignment
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import subprocess
import numpy as np
from typing import Dict

class UnifiedSLAMNode(Node):
    def __init__(self):
        super().__init__('unified_slam')
        
        self.robot_names = ['robot1', 'robot2', 'robot3', 'robot4']
        
        # Subscribe to all robot laser scans
        self.scan_subscribers = []
        for robot_name in self.robot_names:
            sub = self.create_subscription(
                LaserScan,
                f'/{robot_name}/scan',
                lambda msg, rn=robot_name: self.scan_callback(msg, rn),
                10
            )
            self.scan_subscribers.append(sub)
        
        # Republish combined scans to a unified SLAM topic
        self.unified_scan_pub = self.create_publisher(
            LaserScan,
            '/unified_scan',
            10
        )
        
        # Store latest scans from each robot
        self.latest_scans: Dict[str, LaserScan] = {}
        
        # TF broadcaster for robot poses
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Robot spawn positions (known from launch script)
        self.robot_spawn_positions = {
            'robot1': (2.0, 0.0),
            'robot2': (4.0, 0.0),
            'robot3': (6.0, 0.0),
            'robot4': (8.0, 0.0),
        }
        
        self.get_logger().info("Unified Multi-Robot SLAM Node started")
        self.get_logger().info(f"Processing scans from {len(self.robot_names)} robots")
    
    def scan_callback(self, msg: LaserScan, robot_name: str):
        """Store and process laser scan from a robot"""
        self.latest_scans[robot_name] = msg
        
        # For now, just log that we're receiving scans
        # In a full implementation, we'd combine scans and feed to SLAM
        if len(self.latest_scans) == len(self.robot_names):
            self.get_logger().info(
                f"Receiving scans from all {len(self.robot_names)} robots",
                throttle_duration_sec=5.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = UnifiedSLAMNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
