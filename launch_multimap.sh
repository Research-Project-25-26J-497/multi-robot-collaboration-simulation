#!/bin/bash
# Multi-robot mapping with properly namespaced URDFs

NUM_ROBOTS=4
echo "Launching multi-robot SLAM mapping with $NUM_ROBOTS robots..."

source /opt/ros/humble/setup.bash
source ~/multi_robot_ws/install/setup.bash

# Kill any existing processes
killall -9 gzserver gzclient async_slam_toolbox_node robot_state_publisher rviz2 advanced_collaboration_node spawn_entity.py map_fusion web_bridge 2>/dev/null
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

# Launch SLAM for each robot (individual local maps in shared map frame)
echo "Step 4: Launching SLAM nodes for each robot (collaborative mapping)"
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
        -p map_frame:=map &
    
    sleep 2
done

sleep 3

# Publish shared map frame
echo "Step 5: Publishing shared map frame..."
ros2 run tf2_ros static_transform_publisher \
    --frame-id world \
    --child-frame-id map \
    --x 0 --y 0 --z 0 &

sleep 1

# Launch MAP FUSION node with overlap detection and conflict resolution
echo "Step 5b: Launching map fusion node (overlap detection + merging)..."
ros2 run multi_robot_collab map_fusion \
    --ros-args \
    -p use_sim_time:=true > /tmp/map_fusion.log 2>&1 &

sleep 3

# Verify map_fusion is running
if pgrep -f "map_fusion" > /dev/null; then
    echo "✓ Map fusion node started successfully"
else
    echo "✗ WARNING: Map fusion node may not be running!"
    echo "Check /tmp/map_fusion.log for errors"
fi

# Detect Windows host IP when running inside WSL2 so the bridge can reach
# the backend (which runs on Windows).  The host is always the default gateway.
# Falls back to localhost if not WSL2.
if grep -qi 'microsoft' /proc/version 2>/dev/null; then
    WSL2_HOST_IP=$(ip route show | awk '/^default/ {print $3; exit}')
fi
if [ -n "$WSL2_HOST_IP" ]; then
    export MANTIS_BACKEND_URL="http://${WSL2_HOST_IP}:8000"
    echo "  Detected WSL2 – backend URL set to $MANTIS_BACKEND_URL"
else
    export MANTIS_BACKEND_URL="http://localhost:8000"
    echo "  Backend URL: $MANTIS_BACKEND_URL"
fi

# Launch web bridge (forwards SLAM data to the backend at MANTIS_BACKEND_URL)
echo "Step 5c: Launching ROS2-to-Web bridge..."
ros2 run multi_robot_collab web_bridge \
    --ros-args \
    -p use_sim_time:=true > /tmp/web_bridge.log 2>&1 &

sleep 2

if pgrep -f "web_bridge" > /dev/null; then
    echo "✓ Web bridge started — forwarding to $MANTIS_BACKEND_URL"
    echo "  Log: /tmp/web_bridge.log"
else
    echo "✗ WARNING: Web bridge failed to start"
    echo "  Run: pip install requests numpy"
    echo "  Log: /tmp/web_bridge.log"
fi

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
echo "Architecture: Distributed SLAM with Map Fusion + Web Bridge"
echo "Robots: $NUM_ROBOTS (all mapping in shared 'map' frame)"
echo ""
echo "SLAM Visualization Platform:"
echo "  Web bridge: http://localhost:8000"
echo "  Next.js client: cd slam-visualization-platform-client && npm run dev"
echo "  Then open: http://localhost:3000"
echo ""
echo "Maps:"
echo "  Individual: /robot1/map, /robot2/map, /robot3/map, /robot4/map"
echo "  Fused Global: /fused_map (combined with overlap detection)"
echo ""
echo "Map Fusion Features:"
echo "  ✓ Overlap detection between local maps"
echo "  ✓ Conflict resolution via weighted averaging"
echo "  ✓ Incremental updates without interruption"
echo "  ✓ Continuous synchronization across robots"
echo ""
echo "RViz Configuration:"
echo "  1. Fixed Frame: map (shared coordinate system)"
echo "  2. Add -> Map -> Topic: /fused_map (unified global map)"
echo "  OR view individual robot maps:"
echo "  3. Add -> Map -> Topic: /robot1/map"
echo "  4. Add -> Map -> Topic: /robot2/map"
echo "  5. Add -> LaserScan -> Topic: /robot1/scan (red)"
echo "  6. Add -> LaserScan -> Topic: /robot2/scan (green)"
echo "  7. Add -> LaserScan -> Topic: /robot3/scan (blue)"
echo "  8. Add -> LaserScan -> Topic: /robot4/scan (yellow)"
echo ""
echo "To check TF frames:"
echo "  ros2 run tf2_tools view_frames"
echo ""
echo "To save a map:"
echo "  ./save_map.sh"
echo ""
echo "Press Ctrl+C to stop"
echo "========================================="

trap "killall -9 gzserver gzclient async_slam_toolbox_node robot_state_publisher rviz2 advanced_collaboration_node spawn_entity.py static_transform_publisher map_fusion web_bridge 2>/dev/null; exit" INT
wait
