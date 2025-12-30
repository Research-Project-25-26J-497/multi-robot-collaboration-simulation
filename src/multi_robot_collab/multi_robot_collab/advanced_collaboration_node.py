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
        # Exploration goals spread across the environment (expanded for full map)
        self.exploration_goals = [
            (20.0, 20.0), (-20.0, 20.0), (20.0, -20.0), (-20.0, -20.0),
            (0.0, 20.0), (20.0, 0.0), (-20.0, 0.0), (0.0, -20.0)
        ]
        self.assigned_tasks = {}
        # Global map sized for 50x50m area at 0.1m resolution (500x500 cells)
        self.global_map = np.zeros((500, 500))  # 50x50m map at 0.1m resolution
        self.map_origin = (-25, -25)  # Map origin in meters
        
        # Obstacle avoidance parameters
        self.obstacle_detection_range = 2.0  # meters
        self.critical_obstacle_distance = 0.8  # meters - emergency stop
        self.safe_obstacle_distance = 1.2  # meters - start avoiding
        
        # Collaboration timer
        self.collaboration_timer = self.create_timer(1.0, self.run_collaboration_algorithms)
        
        self.get_logger().info("Advanced Multi-Robot Collaboration Framework Started")
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
        """Main collaboration loop implementing task allocation, conflict resolution, and navigation"""
        self.dynamic_task_allocation()
        
        conflicts = self.detect_conflicts()
        if conflicts:
            self.resolve_conflicts(conflicts)
        
        self.detect_and_avoid_obstacles()
        self.fuse_maps()
        self.execute_tasks()
        self.log_system_status()
    
    def dynamic_task_allocation(self):
        """Assign tasks to idle robots based on closest unassigned goal"""
        idle_robots = [name for name in self.robot_names 
                      if self.robot_data[name]['status'] == 'idle']
        unassigned_goals = [goal for goal in self.exploration_goals 
                          if goal not in self.assigned_tasks.values()]
        
        for robot_name in idle_robots:
            if unassigned_goals:
                # Find closest unassigned goal
                robot_pos = self.robot_data[robot_name]['position']
                closest_goal = min(unassigned_goals,
                                 key=lambda goal: self.calculate_distance(robot_pos, goal))
                
                self.assigned_tasks[robot_name] = closest_goal
                self.robot_data[robot_name]['status'] = 'navigating'
                self.robot_data[robot_name]['current_task'] = closest_goal
                unassigned_goals.remove(closest_goal)
                
                self.get_logger().info(f"Task assigned to {robot_name}: {closest_goal}")
    
    def detect_conflicts(self) -> List[Tuple[str, str]]:
        """Detect when robots are too close to each other"""
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
                    self.get_logger().warning(f"Conflict detected: {r1} and {r2} distance {distance:.2f}m")
        
        return conflicts
    
    def resolve_conflicts(self, conflicts: List[Tuple[str, str, float]]):
        """Move robots away from each other when too close"""
        for robot1, robot2, distance in conflicts:
            pos1 = np.array(self.robot_data[robot1]['position'])
            pos2 = np.array(self.robot_data[robot2]['position'])
            
            # Calculate repulsion vector to move robots apart
            repulsion_dir = (pos1 - pos2) / np.linalg.norm(pos1 - pos2)
            
            # Apply avoidance behavior
            avoidance_cmd = Twist()
            avoidance_cmd.linear.x = repulsion_dir[0] * 0.3
            avoidance_cmd.linear.y = repulsion_dir[1] * 0.3
            
            self.robot_data[robot1]['cmd_publisher'].publish(avoidance_cmd)
            self.robot_data[robot2]['cmd_publisher'].publish(avoidance_cmd)
            
            self.get_logger().info(f"Separating {robot1} and {robot2}")
    
    def detect_and_avoid_obstacles(self):
        """Detect and avoid obstacles using LiDAR data"""
        for robot_name in self.robot_names:
            lidar_data = self.robot_data[robot_name]['lidar_data']
            
            if lidar_data is None or not lidar_data.ranges:
                continue
            
            obstacle_info = self.analyze_obstacles(lidar_data)
            
            if obstacle_info['critical_obstacle']:
                # Emergency stop and avoidance
                self.emergency_obstacle_avoidance(robot_name, obstacle_info)
            elif obstacle_info['nearby_obstacle']:
                # Gradual avoidance
                self.gradual_obstacle_avoidance(robot_name, obstacle_info)
    
    def analyze_obstacles(self, lidar_data: LaserScan) -> Dict:
        """Analyze LiDAR data to detect obstacles in different regions"""
        ranges = np.array(lidar_data.ranges)
        angles = np.arange(len(ranges)) * lidar_data.angle_increment + lidar_data.angle_min
        
        # Replace invalid readings with max range
        valid_ranges = np.where(
            (ranges >= lidar_data.range_min) & (ranges <= lidar_data.range_max),
            ranges,
            lidar_data.range_max
        )
        
        # Divide sensor into sectors (front, left, right)
        num_readings = len(valid_ranges)
        front_sector = valid_ranges[num_readings//3:2*num_readings//3]
        left_sector = valid_ranges[2*num_readings//3:]
        right_sector = valid_ranges[:num_readings//3]
        
        # Find minimum distances in each sector
        min_front = np.min(front_sector) if len(front_sector) > 0 else lidar_data.range_max
        min_left = np.min(left_sector) if len(left_sector) > 0 else lidar_data.range_max
        min_right = np.min(right_sector) if len(right_sector) > 0 else lidar_data.range_max
        
        # Find overall minimum and its angle
        min_distance = np.min(valid_ranges)
        min_idx = np.argmin(valid_ranges)
        obstacle_angle = angles[min_idx]
        
        return {
            'critical_obstacle': min_distance < self.critical_obstacle_distance,
            'nearby_obstacle': min_distance < self.safe_obstacle_distance,
            'min_distance': min_distance,
            'obstacle_angle': obstacle_angle,
            'min_front': min_front,
            'min_left': min_left,
            'min_right': min_right,
            'front_blocked': min_front < self.safe_obstacle_distance,
            'left_blocked': min_left < self.safe_obstacle_distance,
            'right_blocked': min_right < self.safe_obstacle_distance
        }
    
    def emergency_obstacle_avoidance(self, robot_name: str, obstacle_info: Dict):
        """Emergency stop and avoidance for critical obstacles"""
        cmd = Twist()
        
        # Stop immediately
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        
        # Turn away from obstacle
        obstacle_angle = obstacle_info['obstacle_angle']
        
        # Turn in opposite direction of obstacle
        if obstacle_angle > 0:  # Obstacle on left, turn right
            cmd.angular.z = -0.5
        else:  # Obstacle on right, turn left
            cmd.angular.z = 0.5
        
        self.robot_data[robot_name]['cmd_publisher'].publish(cmd)
        
        self.get_logger().warning(
            f"EMERGENCY: {robot_name} obstacle at {obstacle_info['min_distance']:.2f}m - stopping and turning"
        )
    
    def gradual_obstacle_avoidance(self, robot_name: str, obstacle_info: Dict):
        """Gradual avoidance for nearby obstacles"""
        # Only apply if robot is navigating
        if self.robot_data[robot_name]['status'] != 'navigating':
            return
        
        cmd = Twist()
        
        # Reduce speed based on obstacle proximity
        distance_factor = (obstacle_info['min_distance'] - self.critical_obstacle_distance) / \
                         (self.safe_obstacle_distance - self.critical_obstacle_distance)
        distance_factor = max(0.0, min(1.0, distance_factor))
        
        # Choose avoidance direction based on which side is more open
        if obstacle_info['front_blocked']:
            # Turn towards more open side
            if obstacle_info['min_left'] > obstacle_info['min_right']:
                cmd.linear.x = 0.1 * distance_factor
                cmd.angular.z = 0.8
                direction = "left"
            else:
                cmd.linear.x = 0.1 * distance_factor
                cmd.angular.z = -0.8
                direction = "right"
            
            self.get_logger().info(
                f"{robot_name} avoiding obstacle at {obstacle_info['min_distance']:.2f}m - turning {direction}"
            )
        else:
            # Slight adjustment, continue forward slowly
            obstacle_angle = obstacle_info['obstacle_angle']
            cmd.linear.x = 0.15 * distance_factor
            cmd.angular.z = -obstacle_angle * 0.5
            
            self.get_logger().info(
                f"{robot_name} adjusting path - obstacle at {obstacle_info['min_distance']:.2f}m"
            )
        
        self.robot_data[robot_name]['cmd_publisher'].publish(cmd)
    
    def fuse_maps(self):
        """Update global map from all robots' LiDAR data"""
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
                    
                    max_x, max_y = self.global_map.shape[0], self.global_map.shape[1]
                    if 0 <= map_x < max_x and 0 <= map_y < max_y:
                        self.global_map[map_x, map_y] = 1
    
    def execute_tasks(self):
        """Execute navigation tasks for each robot"""
        for robot_name in self.robot_names:
            if self.robot_data[robot_name]['status'] == 'navigating':
                # Check for critical obstacles before navigating
                lidar_data = self.robot_data[robot_name]['lidar_data']
                if lidar_data and lidar_data.ranges:
                    obstacle_info = self.analyze_obstacles(lidar_data)
                    if not obstacle_info['critical_obstacle']:
                        self.navigate_to_goal(robot_name)
                else:
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
        
        angle_error = self.normalize_angle(target_angle - current_angle)
        
        cmd = Twist()
        if distance > 0.5:  # Not at goal yet
            cmd.linear.x = min(0.3, distance * 0.5)
            cmd.angular.z = angle_error * 2.0
        else:  # Reached goal
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.robot_data[robot_name]['status'] = 'idle'
            self.get_logger().info(f"{robot_name} reached goal {goal}")
        
        self.robot_data[robot_name]['cmd_publisher'].publish(cmd)
    
    def odom_callback(self, msg: Odometry, robot_name: str):
        """Update robot position from odometry"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Convert quaternion to yaw angle
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
            f"System Status: {active_robots}/{len(self.robot_names)} robots active, "
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