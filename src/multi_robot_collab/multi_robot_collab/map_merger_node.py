#!/usr/bin/env python3
"""
Multi-Robot Map Merger Node
Merges maps from multiple robots into a single unified map
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from typing import Dict, List
import math

class MapMergerNode(Node):
    def __init__(self):
        super().__init__('map_merger')
        
        # Robot names
        self.robot_names = ['robot1', 'robot2', 'robot3', 'robot4']
        
        # Map storage
        self.robot_maps: Dict[str, OccupancyGrid] = {}
        self.last_update_time: Dict[str, float] = {}
        
        # Merged map parameters
        self.merged_map = OccupancyGrid()
        self.merged_map.header.frame_id = 'map'
        self.merged_map.info.resolution = 0.05  # 5cm resolution
        self.merged_map.info.width = 1000  # 50m x 50m map
        self.merged_map.info.height = 1000
        self.merged_map.info.origin.position.x = -25.0
        self.merged_map.info.origin.position.y = -25.0
        self.merged_map.info.origin.position.z = 0.0
        self.merged_map.info.origin.orientation.w = 1.0
        
        # Initialize empty map
        self.merged_map.data = [-1] * (1000 * 1000)
        
        # Subscribe to each robot's map
        self.map_subscribers = []
        for robot_name in self.robot_names:
            sub = self.create_subscription(
                OccupancyGrid,
                f'/{robot_name}/map',
                lambda msg, rn=robot_name: self.map_callback(msg, rn),
                10
            )
            self.map_subscribers.append(sub)
            self.last_update_time[robot_name] = 0.0
        
        # Publisher for merged map
        self.merged_map_pub = self.create_publisher(
            OccupancyGrid,
            '/merged_map',
            10
        )
        
        # Timer to publish merged map
        self.create_timer(1.0, self.publish_merged_map)
        
        self.get_logger().info("Map Merger Node started - merging maps from all robots")
    
    def map_callback(self, msg: OccupancyGrid, robot_name: str):
        """Store incoming map from a robot"""
        self.robot_maps[robot_name] = msg
        self.last_update_time[robot_name] = self.get_clock().now().nanoseconds / 1e9
        
        # Merge maps immediately when new data arrives
        self.merge_maps()
    
    def merge_maps(self):
        """Merge all robot maps into a single unified map"""
        if not self.robot_maps:
            return
        
        # Create merged map array
        merged_width = self.merged_map.info.width
        merged_height = self.merged_map.info.height
        merged_resolution = self.merged_map.info.resolution
        merged_origin_x = self.merged_map.info.origin.position.x
        merged_origin_y = self.merged_map.info.origin.position.y
        
        # Initialize with unknown cells
        merged_data = np.full((merged_height, merged_width), -1, dtype=np.int8)
        
        # Track cell confidence (higher = more reliable)
        cell_confidence = np.zeros((merged_height, merged_width), dtype=np.float32)
        
        # Merge each robot's map
        for robot_name, robot_map in self.robot_maps.items():
            if robot_map is None:
                continue
            
            robot_width = robot_map.info.width
            robot_height = robot_map.info.height
            robot_resolution = robot_map.info.resolution
            robot_origin_x = robot_map.info.origin.position.x
            robot_origin_y = robot_map.info.origin.position.y
            
            # Convert robot map data to numpy array
            robot_data = np.array(robot_map.data).reshape((robot_height, robot_width))
            
            # Iterate through robot map cells
            for ry in range(robot_height):
                for rx in range(robot_width):
                    cell_value = robot_data[ry, rx]
                    
                    # Skip unknown cells
                    if cell_value == -1:
                        continue
                    
                    # Convert robot cell coordinates to world coordinates
                    world_x = robot_origin_x + (rx + 0.5) * robot_resolution
                    world_y = robot_origin_y + (ry + 0.5) * robot_resolution
                    
                    # Convert world coordinates to merged map coordinates
                    mx = int((world_x - merged_origin_x) / merged_resolution)
                    my = int((world_y - merged_origin_y) / merged_resolution)
                    
                    # Check if within merged map bounds
                    if 0 <= mx < merged_width and 0 <= my < merged_height:
                        # Use probabilistic merging
                        # Occupied cells (100) have higher weight than free cells (0)
                        new_confidence = 1.0 if cell_value > 50 else 0.5
                        
                        if cell_confidence[my, mx] == 0:
                            # First time seeing this cell
                            merged_data[my, mx] = cell_value
                            cell_confidence[my, mx] = new_confidence
                        else:
                            # Merge with existing data using weighted average
                            old_value = merged_data[my, mx]
                            old_confidence = cell_confidence[my, mx]
                            
                            # Weight average
                            total_confidence = old_confidence + new_confidence
                            merged_value = (old_value * old_confidence + cell_value * new_confidence) / total_confidence
                            
                            merged_data[my, mx] = int(merged_value)
                            cell_confidence[my, mx] = min(total_confidence, 5.0)  # Cap confidence
        
        # Convert back to list format for ROS message
        self.merged_map.data = merged_data.flatten().tolist()
    
    def publish_merged_map(self):
        """Publish the merged map"""
        self.merged_map.header.stamp = self.get_clock().now().to_msg()
        self.merged_map_pub.publish(self.merged_map)
        
        # Log statistics
        total_cells = len(self.merged_map.data)
        known_cells = sum(1 for cell in self.merged_map.data if cell != -1)
        occupied_cells = sum(1 for cell in self.merged_map.data if cell > 50)
        
        coverage = (known_cells / total_cells) * 100 if total_cells > 0 else 0
        
        self.get_logger().info(
            f"Merged Map: {known_cells}/{total_cells} cells known ({coverage:.1f}%), "
            f"{occupied_cells} occupied | Active robots: {len(self.robot_maps)}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MapMergerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
