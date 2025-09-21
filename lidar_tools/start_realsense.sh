#!/bin/bash
# RealSense D435 Camera Launch Script

echo "=========================================="
echo "Starting RealSense D435 Camera"
echo "=========================================="
echo "Camera: Intel RealSense D435"
echo "Serial: 317622072348"
echo "=========================================="

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Launch RealSense with common parameters
ros2 launch realsense2_camera rs_launch.py \
    depth_module.depth_profile:=640x480x30 \
    rgb_camera.color_profile:=640x480x30 \
    enable_sync:=true \
    align_depth.enable:=true \
    pointcloud.enable:=false