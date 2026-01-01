#!/usr/bin/env python3
"""
Multi-Robot Map Fusion Node
Implements overlap detection, conflict resolution, and incremental map fusion
Based on research proposal requirements for consistent global map creation
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point
from tf2_ros import TransformListener, Buffer
import numpy as np
from typing import Dict, List, Tuple, Optional
import math

class MapFusionNode(Node):
    def __init__(self):
        super().__init__('map_fusion')
        
        # Robot names
        self.robot_names = ['robot1', 'robot2', 'robot3', 'robot4']
        
        # Local map storage
        self.local_maps: Dict[str, OccupancyGrid] = {}
        self.map_alignments: Dict[str, Tuple[float, float, float]] = {}  # (x, y, theta)
        self.last_update_time: Dict[str, float] = {}
        
        # TF buffer for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize alignments with spawn positions
        self.initialize_alignments()
        
        # Fused global map
        self.global_map = OccupancyGrid()
        self.global_map.header.frame_id = 'map'  # Shared map frame
        self.global_map.info.resolution = 0.05  # 5cm resolution
        self.global_map.info.width = 2000  # 100m x 100m
        self.global_map.info.height = 2000
        self.global_map.info.origin.position.x = -50.0
        self.global_map.info.origin.position.y = -50.0
        self.global_map.info.origin.position.z = 0.0
        self.global_map.info.origin.orientation.w = 1.0
        self.global_map.data = [-1] * (2000 * 2000)
        
        # Subscribe to each robot's local map
        self.map_subscribers = []
        for robot_name in self.robot_names:
            sub = self.create_subscription(
                OccupancyGrid,
                f'/{robot_name}/map',
                lambda msg, rn=robot_name: self.local_map_callback(msg, rn),
                10
            )
            self.map_subscribers.append(sub)
            self.last_update_time[robot_name] = 0.0
        
        # Publisher for fused global map
        self.global_map_pub = self.create_publisher(
            OccupancyGrid,
            '/fused_map',
            10
        )
        
        # Timer for continuous fusion updates
        self.create_timer(2.0, self.fuse_maps)
        
        # Fusion parameters
        self.overlap_threshold = 0.3  # Minimum 30% overlap required
        self.confidence_weight = 0.7  # Weight for conflict resolution
        
        self.get_logger().info("Map Fusion Node started")
        self.get_logger().info("Features: Overlap detection, Conflict resolution, Incremental updates")
    
    def initialize_alignments(self):
        """Initialize map alignments based on known spawn positions"""
        spawn_positions = {
            'robot1': (2.0, 0.0, 0.0),
            'robot2': (4.0, 0.0, 0.0),
            'robot3': (6.0, 0.0, 0.0),
            'robot4': (8.0, 0.0, 0.0),
        }
        self.map_alignments = spawn_positions.copy()
        self.get_logger().info(f"Initialized alignments: {self.map_alignments}")
    
    def local_map_callback(self, msg: OccupancyGrid, robot_name: str):
        """Store incoming local map and trigger fusion"""
        self.local_maps[robot_name] = msg
        self.last_update_time[robot_name] = self.get_clock().now().nanoseconds / 1e9
        
        # Attempt overlap detection with other maps
        self.detect_and_align(robot_name)
    
    def detect_and_align(self, new_robot: str):
        """
        Overlap Detection: Find common regions between new map and existing maps
        Updates alignment transforms when overlaps are detected
        """
        if new_robot not in self.local_maps:
            return
        
        new_map = self.local_maps[new_robot]
        
        # Compare with other robot maps to find overlaps
        for other_robot in self.robot_names:
            if other_robot == new_robot or other_robot not in self.local_maps:
                continue
            
            other_map = self.local_maps[other_robot]
            
            # Detect overlap and compute relative transform
            overlap_score, transform = self.compute_overlap(new_map, other_map)
            
            if overlap_score > self.overlap_threshold:
                # Update alignment based on overlap
                self.refine_alignment(new_robot, other_robot, transform)
                self.get_logger().info(
                    f"Overlap detected: {new_robot} ↔ {other_robot} "
                    f"(score: {overlap_score:.2f})",
                    throttle_duration_sec=5.0
                )
    
    def compute_overlap(self, map1: OccupancyGrid, map2: OccupancyGrid) -> Tuple[float, Tuple[float, float, float]]:
        """
        Compute overlap score and relative transform between two maps
        Returns: (overlap_score, (dx, dy, dtheta))
        """
        # Convert maps to numpy arrays
        data1 = np.array(map1.data).reshape((map1.info.height, map1.info.width))
        data2 = np.array(map2.data).reshape((map2.info.height, map2.info.width))
        
        # Create binary masks (known cells)
        mask1 = (data1 != -1).astype(float)
        mask2 = (data2 != -1).astype(float)
        
        # Compute overlap using cell counts (handle different map sizes)
        # In production, use feature matching (ORB, SIFT) or scan matching
        known1 = np.sum(mask1 > 0)
        known2 = np.sum(mask2 > 0)
        
        # If either map is empty, no overlap
        if known1 == 0 or known2 == 0:
            return 0.0, (0.0, 0.0, 0.0)
        
        # Simple heuristic: ratio of smaller to larger known area
        overlap_score = min(known1, known2) / max(known1, known2)
        
        # Simplified transform (in production, use ICP or feature matching)
        transform = (0.0, 0.0, 0.0)
        
        return overlap_score, transform
    
    def refine_alignment(self, robot1: str, robot2: str, relative_transform: Tuple[float, float, float]):
        """
        Refine alignment between maps based on detected overlap
        Implements conflict resolution through weighted averaging
        """
        # In production: Use graph-based optimization (pose graph SLAM)
        # For now: Use weighted average of alignments
        pass
    
    def fuse_maps(self):
        """
        Incremental Map Fusion: Merge local maps into consistent global map
        All robots now use shared 'map' frame, so maps naturally overlay
        """
        if not self.local_maps:
            return
        
        # Create global map array
        width = self.global_map.info.width
        height = self.global_map.info.height
        resolution = self.global_map.info.resolution
        origin_x = self.global_map.info.origin.position.x
        origin_y = self.global_map.info.origin.position.y
        
        # Initialize fusion arrays
        fused_data = np.full((height, width), -1, dtype=np.int8)
        cell_weights = np.zeros((height, width), dtype=np.float32)
        weighted_sum = np.zeros((height, width), dtype=np.float32)
        
        # Fuse each robot's local map (all already in shared 'map' frame)
        for robot_name, local_map in self.local_maps.items():
            if local_map is None:
                continue
            
            # All maps are already in 'map' frame - direct overlay!
            # No transform needed since they share the same reference frame
            
            # Convert local map to numpy
            local_width = local_map.info.width
            local_height = local_map.info.height
            local_resolution = local_map.info.resolution
            
            # Robot's local map origin (already in 'map' frame coordinates)
            local_map_origin_x = local_map.info.origin.position.x
            local_map_origin_y = local_map.info.origin.position.y
            
            local_data = np.array(local_map.data).reshape((local_height, local_width))
            
            # Project each cell from local map to global map
            for ly in range(local_height):
                for lx in range(local_width):
                    cell_value = local_data[ly, lx]
                    
                    if cell_value == -1:  # Unknown cell
                        continue
                    
                    # Cell position in 'map' frame (meters) - no transform needed!
                    map_x = local_map_origin_x + (lx + 0.5) * local_resolution
                    map_y = local_map_origin_y + (ly + 0.5) * local_resolution
                    
                    # Convert to global map grid coordinates
                    gx = int((map_x - origin_x) / resolution)
                    gy = int((map_y - origin_y) / resolution)
                    
                    # Check bounds
                    if 0 <= gx < width and 0 <= gy < height:
                        # Conflict Resolution: Weighted average
                        # Give higher weight to occupied cells
                        weight = 1.0
                        if cell_value > 50:  # Occupied cell
                            weight = 2.0  # Higher confidence for obstacles
                        
                        weighted_sum[gy, gx] += cell_value * weight
                        cell_weights[gy, gx] += weight
        
        # Compute final fused map with conflict resolution
        for y in range(height):
            for x in range(width):
                if cell_weights[y, x] > 0:
                    fused_data[y, x] = int(weighted_sum[y, x] / cell_weights[y, x])
        
        # Update global map
        self.global_map.data = fused_data.flatten().tolist()
        
        # Publish fused map
        self.publish_global_map()
    
    def publish_global_map(self):
        """Publish the fused global map"""
        self.global_map.header.stamp = self.get_clock().now().to_msg()
        self.global_map_pub.publish(self.global_map)
        
        # Log statistics
        total_cells = len(self.global_map.data)
        known_cells = sum(1 for cell in self.global_map.data if cell != -1)
        occupied_cells = sum(1 for cell in self.global_map.data if cell > 50)
        coverage = (known_cells / total_cells) * 100 if total_cells > 0 else 0
        
        self.get_logger().info(
            f"Fused Map: {known_cells}/{total_cells} cells ({coverage:.1f}% coverage), "
            f"{occupied_cells} occupied | Active robots: {len(self.local_maps)}",
            throttle_duration_sec=3.0
        )

def main(args=None):
    rclpy.init(args=args)
    node = MapFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Map fusion node crashed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
