#!/bin/bash
# Complete Robot System Startup Script
# Launches all sensors and robot base for full functionality

echo "=========================================="
echo "🤖 COMPLETE ROBOT SYSTEM STARTUP"
echo "=========================================="
echo "This will start:"
echo "  • Scout Mini robot base (movement)"
echo "  • RSLidar HELIOS 16P"
echo "  • RealSense D435 Camera"
echo "  • Visualization (RViz2)"
echo "=========================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Directory of this script
TOOLS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo ""
echo -e "${YELLOW}Starting in 3 seconds... Press Ctrl+C to abort${NC}"
sleep 3

# Function to check if process is running
check_process() {
    if ps aux | grep -v grep | grep -q "$1"; then
        return 0
    else
        return 1
    fi
}

# Start Robot Base (most critical - needs to be first)
echo ""
echo -e "${GREEN}[1/4] Starting Robot Base...${NC}"
echo "----------------------------------------"
gnome-terminal --title="Scout Mini Robot Base" -- bash -c "$TOOLS_DIR/start_robot_base.sh; exec bash" 2>/dev/null || \
xterm -title "Scout Mini Robot Base" -e "$TOOLS_DIR/start_robot_base.sh" &

sleep 5  # Give robot base time to initialize

# Start LiDAR
echo -e "${GREEN}[2/4] Starting LiDAR...${NC}"
echo "----------------------------------------"
gnome-terminal --title="RSLidar HELIOS 16P" -- bash -c "$TOOLS_DIR/start_lidar.sh; exec bash" 2>/dev/null || \
xterm -title "RSLidar HELIOS 16P" -e "$TOOLS_DIR/start_lidar.sh" &

sleep 3

# Start Camera
echo -e "${GREEN}[3/4] Starting Camera...${NC}"
echo "----------------------------------------"
gnome-terminal --title="RealSense D435" -- bash -c "$TOOLS_DIR/start_realsense.sh; exec bash" 2>/dev/null || \
xterm -title "RealSense D435" -e "$TOOLS_DIR/start_realsense.sh" &

sleep 3

# Start Visualization
echo -e "${GREEN}[4/4] Starting Visualization...${NC}"
echo "----------------------------------------"
gnome-terminal --title="RViz2 Visualization" -- bash -c "$TOOLS_DIR/start_visualization.sh; exec bash" 2>/dev/null || \
xterm -title "RViz2 Visualization" -e "$TOOLS_DIR/start_visualization.sh" &

sleep 5

# Verify everything is running
echo ""
echo "=========================================="
echo "SYSTEM STATUS CHECK"
echo "=========================================="

# Source ROS2 for checks
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Check robot base
if ros2 node list 2>/dev/null | grep -q "scout_cmd_node"; then
    echo -e "✅ Robot Base:    ${GREEN}RUNNING${NC}"
else
    echo -e "❌ Robot Base:    ${RED}NOT FOUND${NC}"
fi

# Check LiDAR
if ros2 topic list 2>/dev/null | grep -q "/rslidar_points"; then
    echo -e "✅ LiDAR:         ${GREEN}RUNNING${NC}"
else
    echo -e "❌ LiDAR:         ${RED}NOT FOUND${NC}"
fi

# Check Camera
if ros2 topic list 2>/dev/null | grep -q "/camera/camera/color/image_raw"; then
    echo -e "✅ Camera:        ${GREEN}RUNNING${NC}"
else
    echo -e "❌ Camera:        ${RED}NOT FOUND${NC}"
fi

# Check RViz
if pgrep -x "rviz2" > /dev/null; then
    echo -e "✅ Visualization: ${GREEN}RUNNING${NC}"
else
    echo -e "⚠️  Visualization: ${YELLOW}LOADING...${NC}"
fi

echo "=========================================="
echo ""
echo "📋 ACTIVE ROS2 TOPICS:"
echo "----------------------------------------"
echo "Movement control: /cmd_vel"
echo "Odometry:        /odom"
echo "LiDAR points:    /rslidar_points"
echo "Camera RGB:      /camera/camera/color/image_raw"
echo "Camera Depth:    /camera/camera/depth/image_rect_raw"
echo ""
echo "=========================================="
echo "✅ ROBOT FULLY OPERATIONAL!"
echo "=========================================="
echo ""
echo "Quick commands:"
echo "  • Teleop:    ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo "  • Stop all:  pkill -f 'ros2|rviz2'"
echo "  • Topics:    ros2 topic list"
echo "  • Nodes:     ros2 node list"
echo ""
echo "All systems launched in separate terminals."
echo "Close this window when done."
echo "=========================================="