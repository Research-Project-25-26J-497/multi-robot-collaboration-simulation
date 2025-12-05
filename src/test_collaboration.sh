#!/bin/bash

echo "🤖 Starting Multi-Robot Collaboration Test..."
echo "=============================================="

# Start Gazebo
echo "Starting Gazebo..."
export LIBGL_ALWAYS_SOFTWARE=1
gazebo ~/empty.world --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so &

echo "Waiting for Gazebo to start..."
sleep 10

# Spawn robots
echo "Spawning robot fleet..."
source /opt/ros/humble/setup.bash
source ~/multi_robot_ws/install/setup.bash

ros2 launch ~/multi_robot_ws/src/spawn_robot.launch.py robot_namespace:=robot1 &
sleep 2
ros2 launch ~/multi_robot_ws/src/spawn_robot.launch.py robot_namespace:=robot2 x:=2.0 &
sleep 2  
ros2 launch ~/multi_robot_ws/src/spawn_robot.launch.py robot_namespace:=robot3 y:=2.0 &
sleep 2

echo "Waiting for robots to initialize..."
sleep 5

# Start collaboration framework
echo "Starting collaboration framework..."
ros2 run multi_robot_collab advanced_collaboration_node &

echo "✅ Multi-robot collaboration test setup complete!"
echo "Robots should now be exploring collaboratively..."
echo "Check Gazebo to see the robots in action!"