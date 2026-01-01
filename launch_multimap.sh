#!/bin/bash
# Multi-robot mapping with properly namespaced URDFs

NUM_ROBOTS=4
echo "Launching multi-robot SLAM mapping with $NUM_ROBOTS robots..."

source /opt/ros/humble/setup.bash
source ~/multi_robot_ws/install/setup.bash

# Kill any existing processes
killall -9 gzserver gzclient async_slam_toolbox_node robot_state_publisher rviz2 advanced_collaboration_node spawn_entity.py 2>/dev/null
sleep 2

# Launch Gazebo
echo "Step 1: Launching Gazebo..."
ros2 launch gazebo_ros gazebo.launch.py world:=/home/sithum_senanayake/multi_robot_ws/src/worlds/multi_room_warehouse.world &
sleep 8

# Generate and spawn robots
echo "Step 2: Spawning $NUM_ROBOTS robots with namespaced URDFs..."
for i in $(seq 1 $NUM_ROBOTS); do
    X_POS=$((i * 2))
    ROBOT_NAME="robot$i"
    
    # Generate namespaced URDF
    ~/multi_robot_ws/generate_robot_urdf.py $ROBOT_NAME > /tmp/${ROBOT_NAME}.urdf
    
    # Spawn in Gazebo
    ros2 run gazebo_ros spawn_entity.py \
        -entity $ROBOT_NAME \
        -file /tmp/${ROBOT_NAME}.urdf \
        -x $X_POS -y 0 -z 0.1 &
    
    sleep 3
done

sleep 5

# Launch robot_state_publisher for each robot
echo "Step 3: Starting robot_state_publishers..."
for i in $(seq 1 $NUM_ROBOTS); do
    ROBOT_NAME="robot$i"
    
    ros2 run robot_state_publisher robot_state_publisher \
        --ros-args \
        -r __ns:=/$ROBOT_NAME \
        -p robot_description:="$(cat /tmp/${ROBOT_NAME}.urdf)" \
        -p use_sim_time:=true &
    
    sleep 1
done

sleep 3

# Launch SLAM for each robot
echo "Step 4: Launching SLAM nodes..."
for i in $(seq 1 $NUM_ROBOTS); do
    ROBOT_NAME="robot$i"
    
    ros2 run slam_toolbox async_slam_toolbox_node \
        --ros-args \
        --params-file ~/multi_robot_ws/install/multi_robot_collab/share/multi_robot_collab/config/mapper_params_online_async.yaml \
        -r scan:=/${ROBOT_NAME}/scan \
        -r map:=/${ROBOT_NAME}/map \
        -r map_metadata:=/${ROBOT_NAME}/map_metadata \
        -p use_sim_time:=true \
        -p odom_frame:=${ROBOT_NAME}/odom \
        -p base_frame:=${ROBOT_NAME}/base_link \
        -p map_frame:=${ROBOT_NAME}/map &
    
    sleep 2
done

sleep 3

# Publish static transforms to connect all robot maps to a world frame
echo "Step 5: Publishing static transforms..."
for i in $(seq 1 $NUM_ROBOTS); do
    ROBOT_NAME="robot$i"
    X_OFFSET=$((i * 2))
    
    ros2 run tf2_ros static_transform_publisher \
        --frame-id world \
        --child-frame-id ${ROBOT_NAME}/map \
        --x 0 --y 0 --z 0 &
    
    sleep 0.5
done

sleep 2

# Launch navigation nodes
echo "Step 6: Starting navigation..."
for i in $(seq 1 $NUM_ROBOTS); do
    ROBOT_NAME="robot$i"
    ros2 run multi_robot_collab advanced_collaboration_node \
        --ros-args \
        -r __ns:=/$ROBOT_NAME &
    sleep 1
done

sleep 2

# Launch RViz
echo "Step 7: Launching RViz..."
ros2 run rviz2 rviz2 &

echo ""
echo "========================================="
echo "✓ Multi-robot SLAM system running!"
echo "========================================="
echo ""
echo "Robots: $NUM_ROBOTS"
echo "Maps: /robot1/map, /robot2/map, /robot3/map, /robot4/map"
echo "Scans: /robot1/scan, /robot2/scan, /robot3/scan, /robot4/scan"
echo ""
echo "RViz Configuration:"
echo "  1. Fixed Frame: world"
echo "  2. Add -> Map -> Topic: /robot1/map"
echo "  3. Add -> Map -> Topic: /robot2/map"
echo "  4. Add -> Map -> Topic: /robot3/map"
echo "  5. Add -> Map -> Topic: /robot4/map"
echo "  4. Add -> LaserScan -> Topic: /robot1/scan"
echo "  5. Add -> LaserScan -> Topic: /robot2/scan"
echo ""
echo "To check TF frames:"
echo "  ros2 run tf2_tools view_frames"
echo ""
echo "To save a map:"
echo "  ./save_map.sh"
echo ""
echo "Press Ctrl+C to stop"
echo "========================================="

trap "killall -9 gzserver gzclient async_slam_toolbox_node robot_state_publisher rviz2 advanced_collaboration_node spawn_entity.py static_transform_publisher 2>/dev/null; exit" INT
wait
