#!/usr/bin/env python3
"""
Advanced Multi-Robot Collaboration with LiDAR Integration
With improved obstacle escape and navigation
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
        self.robot_names = ['robot1', 'robot2', 'robot3', 'robot4']
        self.robot_data = {}
        
        # Initialize robot managers
        self.init_robot_managers()
        
        # Collaboration parameters
        # Safe exploration goals away from walls (25m boundary with 5m margin)
        self.exploration_goals = [
            # Center and inner points
            (0.0, 0.0), (5.0, 5.0), (-5.0, 5.0), (5.0, -5.0), (-5.0, -5.0),
            # Quarter points
            (10.0, 10.0), (-10.0, 10.0), (10.0, -10.0), (-10.0, -10.0),
            # Axis points
            (15.0, 0.0), (-15.0, 0.0), (0.0, 15.0), (0.0, -15.0),
            # Mid points
            (7.0, 7.0), (-7.0, 7.0), (7.0, -7.0), (-7.0, -7.0)
        ]
        
        self.assigned_tasks = {}
        self.visited_goals = set()
        # Global map sized for 50x50m area at 0.1m resolution (500x500 cells)
        self.global_map = np.zeros((500, 500))  # 50x50m map at 0.1m resolution
        self.map_origin = (-25, -25)  # Map origin in meters
        
        # Obstacle avoidance parameters
        self.obstacle_detection_range = 3.0  # meters
        self.critical_obstacle_distance = 0.5  # meters - emergency stop
        self.safe_obstacle_distance = 1.0  # meters - start avoiding
        
        # Navigation parameters
        self.max_linear_speed = 0.2
        self.max_angular_speed = 0.8
        
        # Obstacle escape parameters
        self.escape_timer = {}
        self.escape_duration = 3.0  # seconds to escape before returning to navigation
        self.escape_angle = math.pi / 2  # 90 degrees for escape turn
        
        # Initialize escape timers
        for name in self.robot_names:
            self.escape_timer[name] = 0.0
        
        # Add startup delay
        self.startup_time = time.time()
        self.initialized = False
        
        # Collaboration timer (start after initialization)
        self.create_timer(5.0, self.initialize_system)
        
        self.get_logger().info("Advanced Multi-Robot Collaboration Framework Starting...")
        self.get_logger().info("Initialization in progress...")
    
    def initialize_system(self):
        """Initialize system after startup delay"""
        if not self.initialized:
            self.collaboration_timer = self.create_timer(0.5, self.run_collaboration_algorithms)
            self.initialized = True
            self.get_logger().info("=" * 60)
            self.get_logger().info("SYSTEM INITIALIZED - Starting collaboration")
            self.get_logger().info("=" * 60)
    
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
                'explored_area': 0.0,
                'avoiding_obstacle': False,
                'escape_mode': False,
                'escape_start_time': 0.0,
                'escape_target_angle': 0.0,
                'stuck_counter': 0,
                'last_position': (0.0, 0.0),
                'last_position_time': time.time(),
                'path_blocked_counter': 0,
                'mapping_state': None,
                'mapping_attempts': 0
            }
    
    def run_collaboration_algorithms(self):
        """Main collaboration loop implementing task allocation, conflict resolution, and navigation"""
        if not self.initialized:
            return
            
        self.dynamic_task_allocation()
        
        conflicts = self.detect_conflicts()
        if conflicts:
            self.resolve_conflicts(conflicts)
        
        # First check obstacles and handle escape if needed
        self.detect_and_handle_obstacles()

        # Perform any ongoing mapping behaviors (mapping at goals)
        for name in self.robot_names:
            if self.robot_data[name].get('mapping_state'):
                self.perform_mapping(name)
                # mapping may change status; skip navigation for this robot this cycle
                continue
        
        # Then execute tasks if not escaping
        self.execute_tasks()
        
        self.fuse_maps()
        self.check_stuck_robots()
        self.log_system_status()
    
    def dynamic_task_allocation(self):
        """Assign tasks to idle robots based on closest unassigned goal"""
        idle_robots = [name for name in self.robot_names 
                      if self.robot_data[name]['status'] == 'idle' 
                      and not self.robot_data[name]['escape_mode']]
        unassigned_goals = [goal for goal in self.exploration_goals 
                          if goal not in self.assigned_tasks.values() and goal not in self.visited_goals]
        
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
            avoidance_cmd.linear.x = repulsion_dir[0] * 0.2  # Reduced from 0.3
            avoidance_cmd.linear.y = repulsion_dir[1] * 0.2
            
            self.robot_data[robot1]['cmd_publisher'].publish(avoidance_cmd)
            self.robot_data[robot2]['cmd_publisher'].publish(avoidance_cmd)
            
            self.get_logger().info(f"Separating {robot1} and {robot2}")
    
    def detect_and_handle_obstacles(self):
        """Detect and handle obstacles with escape behavior"""
        current_time = time.time()
        
        for robot_name in self.robot_names:
            lidar_data = self.robot_data[robot_name]['lidar_data']
            
            if lidar_data is None or not lidar_data.ranges:
                continue
            
            obstacle_info = self.analyze_obstacles(lidar_data)
            
            # Check if robot is in escape mode
            if self.robot_data[robot_name]['escape_mode']:
                self.execute_escape_behavior(robot_name, current_time)
                continue
            
            # Check for critical obstacles
            if obstacle_info['critical_obstacle']:
                # Enter escape mode
                self.robot_data[robot_name]['escape_mode'] = True
                self.robot_data[robot_name]['escape_start_time'] = current_time
                self.robot_data[robot_name]['avoiding_obstacle'] = True
                self.robot_data[robot_name]['path_blocked_counter'] += 1
                
                # Choose escape direction based on which side is clearer
                if obstacle_info['min_left'] > obstacle_info['min_right']:
                    escape_direction = 1  # Turn left
                    self.robot_data[robot_name]['escape_target_angle'] = \
                        self.robot_data[robot_name]['orientation'] + self.escape_angle
                else:
                    escape_direction = -1  # Turn right
                    self.robot_data[robot_name]['escape_target_angle'] = \
                        self.robot_data[robot_name]['orientation'] - self.escape_angle
                
                self.get_logger().warning(
                    f"CRITICAL: {robot_name} entering escape mode (obstacle: {obstacle_info['min_distance']:.2f}m)"
                )
                
                # Stop immediately
                cmd = Twist()
                self.robot_data[robot_name]['cmd_publisher'].publish(cmd)
                
            elif obstacle_info['nearby_obstacle']:
                # Gradual avoidance without entering escape mode
                self.gradual_obstacle_avoidance(robot_name, obstacle_info)
                self.robot_data[robot_name]['avoiding_obstacle'] = True
            else:
                # No nearby obstacles
                if self.robot_data[robot_name]['avoiding_obstacle']:
                    self.robot_data[robot_name]['avoiding_obstacle'] = False
                    self.robot_data[robot_name]['path_blocked_counter'] = 0
    
    def execute_escape_behavior(self, robot_name: str, current_time: float):
        """Execute escape behavior to get away from obstacles"""
        escape_start = self.robot_data[robot_name]['escape_start_time']
        escape_elapsed = current_time - escape_start
        
        if escape_elapsed < self.escape_duration:
            # Still in escape mode
            target_angle = self.robot_data[robot_name]['escape_target_angle']
            current_angle = self.robot_data[robot_name]['orientation']
            
            # Calculate angle error
            angle_error = self.normalize_angle(target_angle - current_angle)
            
            cmd = Twist()
            
            if escape_elapsed < 0.5:
                # Phase 1: Back up
                cmd.linear.x = -0.1
                cmd.angular.z = np.sign(angle_error) * 0.3
            elif escape_elapsed < 2.0:
                # Phase 2: Turn toward escape direction
                cmd.linear.x = 0.0
                cmd.angular.z = angle_error * 2.0
            else:
                # Phase 3: Move forward while turning
                cmd.linear.x = 0.1
                cmd.angular.z = angle_error * 0.5
            
            self.robot_data[robot_name]['cmd_publisher'].publish(cmd)
            
            # Check if we've reached target angle
            if abs(angle_error) < 0.2:  # ~11 degrees
                self.robot_data[robot_name]['escape_target_angle'] = \
                    self.robot_data[robot_name]['orientation'] + self.escape_angle / 2
            
        else:
            # Escape mode complete
            self.robot_data[robot_name]['escape_mode'] = False
            self.robot_data[robot_name]['avoiding_obstacle'] = False
            
            # If path was blocked multiple times, consider reassigning task
            if self.robot_data[robot_name]['path_blocked_counter'] > 2:
                self.get_logger().info(f"{robot_name}: Path persistently blocked, will reassign task")
                if robot_name in self.assigned_tasks:
                    del self.assigned_tasks[robot_name]
                self.robot_data[robot_name]['status'] = 'idle'
                self.robot_data[robot_name]['current_task'] = None
            
            self.get_logger().info(f"{robot_name} escape mode complete")
    
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
        
        # Divide sensor into sectors (front, front-left, front-right, left, right)
        num_readings = len(valid_ranges)
        
        # Front sector (45 degrees centered on front)
        front_start = int((num_readings * 0.375))  # 0.375 = 135/360, center is at 180
        front_end = int((num_readings * 0.625))
        front_sector = valid_ranges[front_start:front_end]
        
        # Left sector (45 degrees)
        left_start = int((num_readings * 0.625))
        left_end = int((num_readings * 0.875))
        left_sector = valid_ranges[left_start:left_end]
        
        # Right sector (45 degrees)
        right_start = int((num_readings * 0.125))
        right_end = int((num_readings * 0.375))
        right_sector = valid_ranges[right_start:right_end]
        
        # Find minimum distances in each sector
        min_front = np.min(front_sector) if len(front_sector) > 0 else lidar_data.range_max
        min_left = np.min(left_sector) if len(left_sector) > 0 else lidar_data.range_max
        min_right = np.min(right_sector) if len(right_sector) > 0 else lidar_data.range_max
        
        # Find overall minimum in front 180 degrees
        front_180 = valid_ranges[int(num_readings * 0.25):int(num_readings * 0.75)]
        min_distance = np.min(front_180) if len(front_180) > 0 else lidar_data.range_max
        
        return {
            'critical_obstacle': min_distance < self.critical_obstacle_distance,
            'nearby_obstacle': min_distance < self.safe_obstacle_distance,
            'min_distance': min_distance,
            'min_front': min_front,
            'min_left': min_left,
            'min_right': min_right,
            'front_blocked': min_front < self.safe_obstacle_distance,
            'left_blocked': min_left < self.safe_obstacle_distance * 1.5,
            'right_blocked': min_right < self.safe_obstacle_distance * 1.5
        }
    
    def gradual_obstacle_avoidance(self, robot_name: str, obstacle_info: Dict):
        """Gradual avoidance for nearby obstacles"""
        # Don't avoid if in escape mode
        if self.robot_data[robot_name]['escape_mode']:
            return
        
        cmd = Twist()
        
        # Reduce speed based on obstacle proximity
        if obstacle_info['min_distance'] < self.safe_obstacle_distance:
            distance_factor = (obstacle_info['min_distance'] - self.critical_obstacle_distance) / \
                             (self.safe_obstacle_distance - self.critical_obstacle_distance)
            distance_factor = max(0.1, min(1.0, distance_factor))  # Minimum 10% speed
        else:
            distance_factor = 1.0
        
        # Choose avoidance direction
        if obstacle_info['front_blocked']:
            # Front blocked, need to turn
            if obstacle_info['min_left'] > obstacle_info['min_right']:
                # More space on left, turn left
                cmd.linear.x = 0.08 * distance_factor
                cmd.angular.z = 0.4
            else:
                # More space on right, turn right
                cmd.linear.x = 0.08 * distance_factor
                cmd.angular.z = -0.4
            
            self.get_logger().info(
                f"{robot_name} avoiding obstacle - turning (dist: {obstacle_info['min_distance']:.2f}m)"
            )
        else:
            # Path relatively clear, proceed with caution
            cmd.linear.x = 0.12 * distance_factor
            cmd.angular.z = 0.0
        
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
            # Skip if escaping or avoiding obstacles
            if (self.robot_data[robot_name]['escape_mode'] or 
                self.robot_data[robot_name]['avoiding_obstacle']):
                continue

            # If mapping in progress, let mapping handler run
            if self.robot_data[robot_name].get('mapping_state'):
                continue

            if self.robot_data[robot_name]['status'] == 'navigating':
                # Check LiDAR before navigating
                lidar_data = self.robot_data[robot_name]['lidar_data']
                if lidar_data and lidar_data.ranges:
                    obstacle_info = self.analyze_obstacles(lidar_data)
                    # Only navigate if path is relatively clear
                    if (not obstacle_info['critical_obstacle'] and 
                        not obstacle_info['front_blocked']):
                        self.navigate_to_goal(robot_name)
                    else:
                        # Path is blocked, stop and let obstacle handler deal with it
                        cmd = Twist()
                        self.robot_data[robot_name]['cmd_publisher'].publish(cmd)
                else:
                    self.navigate_to_goal(robot_name)
    
    def navigate_to_goal(self, robot_name: str):
        """Improved navigation to assigned goal with obstacle awareness"""
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
        
        if distance > 0.3:  # Not at goal yet
            # Adaptive speed control
            # Slow down when close to goal or when angle error is large
            linear_speed = min(self.max_linear_speed, distance * 0.3)
            
            if abs(angle_error) > math.pi/4:  # More than 45 degrees off
                linear_speed *= 0.5  # Reduce speed when turning sharply
            
            # Smooth angular control with limits
            angular_gain = 1.5
            angular_speed = angle_error * angular_gain
            angular_speed = max(-self.max_angular_speed, 
                               min(self.max_angular_speed, angular_speed))
            
            cmd.linear.x = linear_speed
            cmd.angular.z = angular_speed
        else:  # Reached goal
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            # Enter mapping phase instead of immediately marking goal complete
            self.start_mapping(robot_name, goal)
        
        self.robot_data[robot_name]['cmd_publisher'].publish(cmd)
    
    def check_stuck_robots(self):
        """Check if robots are stuck and help them recover"""
        current_time = time.time()
        for robot_name in self.robot_names:
            if self.robot_data[robot_name]['status'] == 'navigating':
                current_pos = self.robot_data[robot_name]['position']
                last_pos = self.robot_data[robot_name]['last_position']
                last_time = self.robot_data[robot_name]['last_position_time']
                
                # Check if robot hasn't moved much
                distance_moved = self.calculate_distance(current_pos, last_pos)
                time_elapsed = current_time - last_time
                
                if time_elapsed > 3.0 and distance_moved < 0.1:
                    self.robot_data[robot_name]['stuck_counter'] += 1
                    
                    if self.robot_data[robot_name]['stuck_counter'] > 1:
                        # Robot appears stuck
                        self.get_logger().warning(f"{robot_name} appears stuck, forcing escape mode")
                        
                        # Force escape mode
                        self.robot_data[robot_name]['escape_mode'] = True
                        self.robot_data[robot_name]['escape_start_time'] = current_time
                        self.robot_data[robot_name]['avoiding_obstacle'] = True
                        
                        # Random escape direction
                        if np.random.random() > 0.5:
                            self.robot_data[robot_name]['escape_target_angle'] = \
                                self.robot_data[robot_name]['orientation'] + self.escape_angle
                        else:
                            self.robot_data[robot_name]['escape_target_angle'] = \
                                self.robot_data[robot_name]['orientation'] - self.escape_angle
                        
                        # Reset stuck counter
                        self.robot_data[robot_name]['stuck_counter'] = 0
                
                # Update last position
                self.robot_data[robot_name]['last_position'] = current_pos
                self.robot_data[robot_name]['last_position_time'] = current_time

    def start_mapping(self, robot_name: str, goal: Tuple[float, float]):
        """Begin local mapping behavior at a goal: rotate in place to scan surroundings."""
        robot = self.robot_data[robot_name]
        robot['status'] = 'mapping'
        robot['mapping_state'] = {
            'phase': 'rotating',
            'start_time': time.time(),
            'duration': 4.0,  # rotate for 4 seconds
            'goal': goal
        }
        robot['mapping_attempts'] = robot.get('mapping_attempts', 0) + 1
        self.get_logger().info(f"{robot_name} starting mapping at goal {goal} (attempt {robot['mapping_attempts']})")

    def perform_mapping(self, robot_name: str):
        """Run mapping rotation and evaluate coverage around the goal."""
        robot = self.robot_data[robot_name]
        state = robot.get('mapping_state')
        if not state:
            return

        elapsed = time.time() - state['start_time']

        if state['phase'] == 'rotating':
            # Rotate in place to gather LiDAR
            if elapsed < state['duration']:
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.6
                robot['cmd_publisher'].publish(cmd)
                return
            else:
                # Rotation done; evaluate map coverage
                state['phase'] = 'evaluating'

        if state['phase'] == 'evaluating':
            goal = state['goal']
            covered = self.check_goal_coverage(goal, radius_m=2.0)
            # If coverage sufficient or too many attempts, mark visited
            if covered >= 0.6 or robot['mapping_attempts'] >= 2:
                self.visited_goals.add(goal)
                if robot_name in self.assigned_tasks:
                    del self.assigned_tasks[robot_name]
                robot['current_task'] = None
                robot['mapping_state'] = None
                robot['status'] = 'idle'
                self.get_logger().info(f"{robot_name} completed mapping at {goal} (coverage {covered:.2f})")
            else:
                # Not enough coverage: if attempts left, try repositioning slightly and retry
                if robot['mapping_attempts'] < 2:
                    robot['mapping_state'] = None
                    robot['status'] = 'navigating'
                    # nudge robot slightly forward to get new view
                    cmd = Twist()
                    cmd.linear.x = 0.08
                    robot['cmd_publisher'].publish(cmd)
                    self.get_logger().info(f"{robot_name} insufficient coverage ({covered:.2f}), repositioning and retrying")
                else:
                    # Give up and mark visited to avoid loops
                    self.visited_goals.add(goal)
                    if robot_name in self.assigned_tasks:
                        del self.assigned_tasks[robot_name]
                    robot['current_task'] = None
                    robot['mapping_state'] = None
                    robot['status'] = 'idle'
                    self.get_logger().info(f"{robot_name} gave up mapping at {goal} after attempts (coverage {covered:.2f})")

    def check_goal_coverage(self, goal: Tuple[float, float], radius_m: float = 2.0) -> float:
        """Return fraction of cells within radius around goal that are marked explored in global_map."""
        gx, gy = goal
        # Convert world coords to map indices
        map_x = int((gx - self.map_origin[0]) / 0.1)
        map_y = int((gy - self.map_origin[1]) / 0.1)
        max_x, max_y = self.global_map.shape

        r_cells = int(radius_m / 0.1)
        xs = range(max(0, map_x - r_cells), min(max_x, map_x + r_cells + 1))
        ys = range(max(0, map_y - r_cells), min(max_y, map_y + r_cells + 1))

        total = 0
        marked = 0
        for ix in xs:
            for iy in ys:
                dx = (ix - map_x)
                dy = (iy - map_y)
                if dx*dx + dy*dy <= r_cells*r_cells:
                    total += 1
                    if self.global_map[ix, iy] > 0:
                        marked += 1

        if total == 0:
            return 0.0
        return marked / total
    
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
        active_robots = len([r for r in self.robot_data.values() if r['status'] == 'navigating'])
        avoiding_robots = len([r for r in self.robot_data.values() if r['avoiding_obstacle']])
        escaping_robots = len([r for r in self.robot_data.values() if r['escape_mode']])
        idle_robots = len([r for r in self.robot_data.values() if r['status'] == 'idle'])
        explored_cells = np.sum(self.global_map > 0)
        
        self.get_logger().info(
            f"Status: {active_robots} navigating, {avoiding_robots} avoiding, "
            f"{escaping_robots} escaping, {idle_robots} idle | "
            f"Tasks: {len(self.assigned_tasks)} assigned | "
            f"Explored: {explored_cells} cells"
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