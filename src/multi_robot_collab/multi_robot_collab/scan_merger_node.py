#!/usr/bin/env python3
"""
Multi-Robot Scan Merger Node
Combines laser scans from multiple robots into a single scan for unified SLAM
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
import numpy as np
import math
from typing import Dict, Optional

class ScanMergerNode(Node):
    def __init__(self):
        super().__init__('scan_merger')
        
        self.robot_names = ['robot1', 'robot2', 'robot3', 'robot4']
        self.latest_scans: Dict[str, LaserScan] = {}
        
        # TF buffer for lookups
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to each robot's scan
        self.scan_subs = []
        for robot_name in self.robot_names:
            sub = self.create_subscription(
                LaserScan,
                f'/{robot_name}/scan',
                lambda msg, rn=robot_name: self.scan_callback(msg, rn),
                10
            )
            self.scan_subs.append(sub)
        
        # Publish merged scan
        self.merged_scan_pub = self.create_publisher(
            LaserScan,
            '/merged_scan',
            10
        )
        
        # Timer to publish merged scans
        self.create_timer(0.1, self.publish_merged_scan)
        
        # Broadcast static transforms for non-robot1 robots to robot1's odom
        self.robot_offsets = {
            'robot2': 2.0,  # robot2 is 2m ahead of robot1
            'robot3': 4.0,
            'robot4': 6.0,
        }
        
        # Publish static TF transforms
        self.create_timer(0.1, self.publish_static_transforms)
        
        self.get_logger().info("Scan Merger Node started - combining scans from all robots")
    
    def publish_static_transforms(self):
        """Publish static transforms from robot1/odom to other robot odoms"""
        for robot_name, x_offset in self.robot_offsets.items():
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'robot1/odom'
            t.child_frame_id = f'{robot_name}/odom'
            t.transform.translation.x = x_offset
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0
            
            self.tf_broadcaster.sendTransform(t)
    
    def scan_callback(self, msg: LaserScan, robot_name: str):
        """Store incoming scan"""
        self.latest_scans[robot_name] = msg
    
    def publish_merged_scan(self):
        """Merge all scans and publish as a single scan from robot1's perspective"""
        if 'robot1' not in self.latest_scans:
            return
        
        # Use robot1's scan as the base
        base_scan = self.latest_scans['robot1']
        merged_scan = LaserScan()
        merged_scan.header = base_scan.header
        merged_scan.header.frame_id = 'robot1/base_link'
        merged_scan.angle_min = base_scan.angle_min
        merged_scan.angle_max = base_scan.angle_max
        merged_scan.angle_increment = base_scan.angle_increment
        merged_scan.time_increment = base_scan.time_increment
        merged_scan.scan_time = base_scan.scan_time
        merged_scan.range_min = base_scan.range_min
        merged_scan.range_max = base_scan.range_max
        
        # Start with robot1's ranges
        merged_ranges = list(base_scan.ranges)
        
        # Merge other robots' scans
        for robot_name in ['robot2', 'robot3', 'robot4']:
            if robot_name not in self.latest_scans:
                continue
            
            robot_scan = self.latest_scans[robot_name]
            
            # Simple merge: take the minimum range at each angle
            # (closer obstacle wins)
            for i in range(min(len(merged_ranges), len(robot_scan.ranges))):
                r1 = merged_ranges[i]
                r2 = robot_scan.ranges[i] + self.robot_offsets.get(robot_name, 0.0)
                
                if math.isinf(r1) or math.isnan(r1):
                    merged_ranges[i] = r2
                elif not (math.isinf(r2) or math.isnan(r2)):
                    merged_ranges[i] = min(r1, r2)
        
        merged_scan.ranges = merged_ranges
        merged_scan.intensities = base_scan.intensities
        
        self.merged_scan_pub.publish(merged_scan)

def main(args=None):
    rclpy.init(args=args)
    node = ScanMergerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
