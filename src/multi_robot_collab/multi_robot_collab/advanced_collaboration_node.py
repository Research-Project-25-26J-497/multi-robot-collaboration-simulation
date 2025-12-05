#!/usr/bin/env python3
"""
Advanced Multi-Robot Collaboration with LiDAR Integration
Implements the core algorithms from your research proposal
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from typing import List, Dict, Tuple
import time

class AdvancedCollaborationNode(Node):
    def __init__(self):
        super().__init__('advanced_collaboration_node')
        
        # Robot fleet configuration
        self.robot_names = ['robot1', 'robot2', 'robot3']
        self.robot_data = {}
        
        # Initialize robot managers
        self.init_robot_managers()
        
        # Collaboration parameters
        self.exploration_goals = [
            (5.0, 5.0), (-5.0, 5.0), (5.0, -5.0), (-5.0, -5.0),
            (8.0, 0.0), (0.0, 8.0), (-8.0, 0.0), (0.0, -8.0)
        ]
        self.assigned_tasks = {}
        self.global_map = np.zeros((200, 200))  # 20x20m map at 0.1m resolution
        self.map_origin = (-10, -10)  # Map origin in meters
        
        # Collaboration timer
        self.collaboration_timer = self.create_timer(1.0, self.run_collaboration_algorithms)
        
        self.get_logger().info("🤖 Advanced Multi-Robot Collaboration Framework Started!")
        self.get_logger().info(f"Managing {len(self.robot_names)} robots with LiDAR")
    
    def init_robot_managers(self):
        """Initialize robot managers with publishers and subscribers"""
        for name in self.robot_names:
            # Command publishers
            cmd_pub = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            
            # Odometry subscribers
            self.create_subscription(
                Odometry, f'/{name}/odom',
                lambda msg, rn=name: self.odom_callback(msg, rn), 10
            )
            
            # LiDAR subscribers  
            self.create_subscription(
                LaserScan, f'/{name}/scan',
                lambda msg, rn=name: self.lidar_callback(msg, rn), 10
            )
            
            self.robot_data[name] = {
                'cmd_publisher': cmd_pub,
                'position': (0.0, 0.0),
                'orientation': 0.0,
                'lidar_data': None,
                'status': 'idle',
                'current_task': None,
                'explored_area': 0.0
            }
    
    def run_collaboration_algorithms(self):
        """Main collaboration loop - implements your research algorithms"""
        # 1. Dynamic Task Allocation
        self.dynamic_task_allocation()
        
        # 2. Conflict Detection and Resolution
        conflicts = self.detect_conflicts()
        if conflicts:
            self.resolve_conflicts(conflicts)
        
        # 3. Map Fusion from LiDAR data
        self.fuse_maps()
        
        # 4. Execute assigned tasks
        self.execute_tasks()
        
        # 5. Log system status
        self.log_system_status()
    
    def dynamic_task_allocation(self):
        """YOUR ALGORITHM: Dynamic task allocation based on robot positions and capabilities"""
        idle_robots = [name for name in self.robot_names 
                      if self.robot_data[name]['status'] == 'idle']
        unassigned_goals = [goal for goal in self.exploration_goals 
                          if goal not in self.assigned_tasks.values()]
        
        for robot_name in idle_robots:
            if unassigned_goals:
                # Find closest unassigned goal (greedy allocation)
                robot_pos = self.robot_data[robot_name]['position']
                closest_goal = min(unassigned_goals,
                                 key=lambda goal: self.calculate_distance(robot_pos, goal))
                
                # Assign task
                self.assigned_tasks[robot_name] = closest_goal
                self.robot_data[robot_name]['status'] = 'navigating'
                self.robot_data[robot_name]['current_task'] = closest_goal
                unassigned_goals.remove(closest_goal)
                
                self.get_logger().info(f"📋 Task Allocation: {robot_name} → {closest_goal}")
    
    def detect_conflicts(self) -> List[Tuple[str, str]]:
        """YOUR ALGORITHM: Conflict detection between robots"""
        conflicts = []
        robot_list = list(self.robot_data.keys())
        
        for i in range(len(robot_list)):
            for j in range(i + 1, len(robot_list)):
                r1 = robot_list[i]
                r2 = robot_list[j]
                pos1 = self.robot_data[r1]['position']
                pos2 = self.robot_data[r2]['position']
                
                distance = self.calculate_distance(pos1, pos2)
                
                # Conflict if robots are too close
                if distance < 1.0:
                    conflicts.append((r1, r2, distance))
                    self.get_logger().warning(f"⚠️ Conflict: {r1} and {r2} too close ({distance:.2f}m)")
        
        return conflicts
    
    def resolve_conflicts(self, conflicts: List[Tuple[str, str, float]]):
        """YOUR ALGORITHM: Conflict resolution strategies"""
        for robot1, robot2, distance in conflicts:
            # Simple resolution: make robots move away from each other
            pos1 = np.array(self.robot_data[robot1]['position'])
            pos2 = np.array(self.robot_data[robot2]['position'])
            
            # Calculate repulsion vector
            repulsion_dir = (pos1 - pos2) / np.linalg.norm(pos1 - pos2)
            
            # Apply temporary avoidance behavior
            avoidance_cmd = Twist()
            avoidance_cmd.linear.x = repulsion_dir[0] * 0.3
            avoidance_cmd.linear.y = repulsion_dir[1] * 0.3
            
            self.robot_data[robot1]['cmd_publisher'].publish(avoidance_cmd)
            self.robot_data[robot2]['cmd_publisher'].publish(avoidance_cmd)
            
            self.get_logger().info(f"🔄 Conflict Resolution: Separating {robot1} and {robot2}")
    
    def fuse_maps(self):
        """YOUR ALGORITHM: Map fusion from individual robot LiDAR data"""
        for robot_name in self.robot_names:
            if self.robot_data[robot_name]['lidar_data']:
                self.update_global_map(robot_name)
    
    def update_global_map(self, robot_name: str):
        """Update global map with robot's LiDAR data"""
        lidar_data = self.robot_data[robot_name]['lidar_data']
        robot_pos = self.robot_data[robot_name]['position']
        robot_angle = self.robot_data[robot_name]['orientation']
        
        if lidar_data.ranges:
            for i, distance in enumerate(lidar_data.ranges):
                if lidar_data.range_min < distance < lidar_data.range_max:
                    # Calculate obstacle position in global frame
                    angle = lidar_data.angle_min + i * lidar_data.angle_increment + robot_angle
                    obstacle_x = robot_pos[0] + distance * math.cos(angle)
                    obstacle_y = robot_pos[1] + distance * math.sin(angle)
                    
                    # Update global map
                    map_x = int((obstacle_x - self.map_origin[0]) / 0.1)
                    map_y = int((obstacle_y - self.map_origin[1]) / 0.1)
                    
                    if 0 <= map_x < 200 and 0 <= map_y < 200:
                        self.global_map[map_x, map_y] = 1  # Mark as occupied
    
    def execute_tasks(self):
        """Execute navigation tasks for each robot"""
        for robot_name in self.robot_names:
            if self.robot_data[robot_name]['status'] == 'navigating':
                self.navigate_to_goal(robot_name)
    
    def navigate_to_goal(self, robot_name: str):
        """Simple navigation to assigned goal"""
        goal = self.robot_data[robot_name]['current_task']
        current_pos = self.robot_data[robot_name]['position']
        current_angle = self.robot_data[robot_name]['orientation']
        
        # Calculate direction to goal
        dx = goal[0] - current_pos[0]
        dy = goal[1] - current_pos[1]
        target_angle = math.atan2(dy, dx)
        distance = math.sqrt(dx**2 + dy**2)
        
        # Simple PD controller
        angle_error = self.normalize_angle(target_angle - current_angle)
        
        cmd = Twist()
        if distance > 0.5:  # Not at goal
            cmd.linear.x = min(0.3, distance * 0.5)  # Proportional speed
            cmd.angular.z = angle_error * 2.0  # Proportional turning
        else:  # Reached goal
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.robot_data[robot_name]['status'] = 'idle'
            self.get_logger().info(f"🎯 {robot_name} reached goal {goal}")
        
        self.robot_data[robot_name]['cmd_publisher'].publish(cmd)
    
    def odom_callback(self, msg: Odometry, robot_name: str):
        """Update robot position from odometry"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Convert quaternion to Euler angle (yaw)
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.robot_data[robot_name]['position'] = (position.x, position.y)
        self.robot_data[robot_name]['orientation'] = yaw
    
    def lidar_callback(self, msg: LaserScan, robot_name: str):
        """Update LiDAR data"""
        self.robot_data[robot_name]['lidar_data'] = msg
    
    def calculate_distance(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
        """Calculate Euclidean distance between two points"""
        return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)
    
    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def log_system_status(self):
        """Log current system status"""
        active_robots = len([r for r in self.robot_data.values() if r['status'] != 'idle'])
        explored_cells = np.sum(self.global_map > 0)
        
        self.get_logger().info(
            f"📊 System Status: {active_robots}/{len(self.robot_names)} robots active, "
            f"{explored_cells} map cells explored"
        )

def main():
    rclpy.init()
    node = AdvancedCollaborationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down advanced collaboration framework...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()