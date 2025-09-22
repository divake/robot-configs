#!/bin/bash
# Scout Mini Robot Base Startup Script
# This script handles all initialization for robot movement

echo "=========================================="
echo "Starting Scout Mini Robot Base"
echo "=========================================="
echo "Robot: AgileX Scout Mini Pro"
echo "Interface: CAN bus (can0)"
echo "=========================================="

# Check if CAN interface exists
if ! ip link show can0 &> /dev/null; then
    echo "❌ ERROR: CAN interface can0 not found!"
    echo "Make sure the CAN adapter is connected"
    exit 1
fi

# Check CAN interface status
CAN_STATUS=$(ip link show can0 | grep -o "state [A-Z]*" | cut -d' ' -f2)

if [ "$CAN_STATUS" != "UP" ]; then
    echo "📡 CAN interface is DOWN. Activating..."
    sudo ip link set can0 up type can bitrate 500000

    # Verify it's up
    sleep 1
    CAN_STATUS=$(ip link show can0 | grep -o "state [A-Z]*" | cut -d' ' -f2)

    if [ "$CAN_STATUS" = "UP" ]; then
        echo "✅ CAN interface activated successfully"
    else
        echo "❌ Failed to activate CAN interface"
        echo "Try manually: sudo ip link set can0 up type can bitrate 500000"
        exit 1
    fi
else
    echo "✅ CAN interface already UP"
fi

# Display CAN status
echo ""
echo "CAN Interface Status:"
ip link show can0
echo ""

# Source ROS2 environment
echo "🤖 Setting up ROS2 environment..."
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Check if node is already running
if ros2 node list | grep -q "scout_cmd_node"; then
    echo "⚠️  Scout driver already running. Stopping it first..."
    # Find and kill the existing process
    pkill -f "scout_cmd scout_mini.launch.py"
    sleep 2
fi

# Launch the Scout Mini driver
echo ""
echo "🚀 Launching Scout Mini driver..."
echo "=========================================="

ros2 launch scout_cmd scout_mini.launch.py &
SCOUT_PID=$!

# Wait for node to initialize
echo "Waiting for driver to initialize..."
sleep 3

# Verify the node is running
if ros2 node list | grep -q "scout_cmd_node"; then
    echo ""
    echo "✅ Scout Mini driver running successfully!"
    echo "=========================================="
    echo ""
    echo "📊 Active topics:"
    ros2 topic list | grep -E "(cmd_vel|odom|tf)"
    echo ""
    echo "=========================================="
    echo "✅ ROBOT BASE READY!"
    echo "=========================================="
    echo ""
    echo "Control options:"
    echo "  1. Keyboard: ros2 run teleop_twist_keyboard teleop_twist_keyboard"
    echo "  2. Commands: ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'"
    echo "  3. Your code: Publish to /cmd_vel topic"
    echo ""
    echo "To stop: Ctrl+C or kill process $SCOUT_PID"
    echo "=========================================="

    # Keep the script running
    wait $SCOUT_PID
else
    echo "❌ Failed to start Scout Mini driver"
    echo "Check the error messages above"
    kill $SCOUT_PID 2>/dev/null
    exit 1
fi