#!/bin/bash
# Script to verify LiDAR sensor is working

echo "Waiting for topics to be available..."
sleep 5

echo "Checking for /robot1/scan topic..."
ros2 topic list | grep "/robot1/scan"

if [ $? -eq 0 ]; then
    echo "✓ /robot1/scan topic found!"
    echo ""
    echo "Getting topic info..."
    ros2 topic info /robot1/scan
    echo ""
    echo "Echoing first message from /robot1/scan..."
    timeout 5 ros2 topic echo /robot1/scan --once
else
    echo "✗ /robot1/scan topic not found"
    echo ""
    echo "Available topics:"
    ros2 topic list
fi
