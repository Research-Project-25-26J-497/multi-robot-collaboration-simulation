#!/bin/bash
# Quick script to save the current map

source /opt/ros/humble/setup.bash
source ~/multi_robot_ws/install/setup.bash

echo "Checking available map topics..."
ros2 topic list | grep "^/.*map$"
echo ""

read -p "Enter map topic to save (e.g., /map or /robot1/map): " MAP_TOPIC
MAP_TOPIC=${MAP_TOPIC:-/map}

MAP_NAME="warehouse_map"
read -p "Enter map filename [warehouse_map]: " USER_MAP_NAME
MAP_NAME=${USER_MAP_NAME:-$MAP_NAME}

# Create map_images directory if it doesn't exist
mkdir -p map_images

echo ""
echo "Saving map from topic '$MAP_TOPIC' to '$MAP_NAME'..."
ros2 run nav2_map_server map_saver_cli -f map_images/$MAP_NAME --ros-args -r map:=$MAP_TOPIC -p use_sim_time:=true

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Map saved successfully!"
    echo "  - map_images/${MAP_NAME}.pgm (image file)"
    echo "  - map_images/${MAP_NAME}.yaml (metadata)"
    echo ""
    
    # Convert to PNG and JPEG
    if command -v convert &> /dev/null; then
        echo "Converting to PNG and JPEG formats..."
        convert map_images/${MAP_NAME}.pgm map_images/${MAP_NAME}.png
        convert map_images/${MAP_NAME}.pgm map_images/${MAP_NAME}.jpg
        echo "  - map_images/${MAP_NAME}.png (PNG format)"
        echo "  - map_images/${MAP_NAME}.jpg (JPEG format)"
    else
        echo "Install ImageMagick to auto-convert: sudo apt install imagemagick"
    fi
    
    echo ""
    echo "Maps saved in: $(pwd)/map_images"
    echo "View with: eog map_images/${MAP_NAME}.png"
else
    echo ""
    echo "✗ Failed to save map. Make sure:"
    echo "  - SLAM is running (ros2 node list | grep slam)"
    echo "  - Map topic exists (ros2 topic list | grep map)"
    echo "  - Map is being published (ros2 topic echo $MAP_TOPIC --once)"
fi
