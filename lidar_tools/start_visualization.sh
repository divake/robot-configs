#!/bin/bash
# Complete Visualization Setup Script
# This starts everything needed for RViz2 to work properly

echo "=========================================="
echo "Starting Visualization Environment"
echo "=========================================="

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Create the transform between map and rslidar frames
# This is REQUIRED for RViz2 to work properly
echo "Setting up coordinate frame transforms..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map rslidar &
TF_PID=$!
echo "Transform publisher started (PID: $TF_PID)"

# Give it a moment to start
sleep 1

# Launch RViz2
echo "Launching RViz2..."
echo ""
echo "IMPORTANT REMINDERS:"
echo "1. Make sure LiDAR and Camera are already running"
echo "2. Fixed Frame should be set to 'rslidar'"
echo "3. Add displays: PointCloud2 for /rslidar_points"
echo "4. Add displays: Image for /camera/camera/color/image_raw"
echo ""

# Check if custom config exists
if [ -f ~/.rviz2/default.rviz ]; then
    echo "Loading your saved RViz2 configuration..."
    rviz2
else
    echo "Loading default configuration..."
    rviz2
fi

# When RViz2 closes, clean up the transform publisher
echo "Cleaning up..."
kill $TF_PID 2>/dev/null