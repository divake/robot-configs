#!/bin/bash
# RSLidar Auto-Start Script
# This will work every time because your IP is now static at 192.168.1.244

echo "=========================================="
echo "Starting RSLidar Driver"
echo "=========================================="
echo "Host IP: 192.168.1.244 (static)"
echo "LiDAR IP: 192.168.1.200"
echo "=========================================="

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch RSLidar
ros2 launch rslidar_sdk start.py