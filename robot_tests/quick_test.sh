#!/bin/bash
# Quick Robot Movement Test Script
# Tests basic movements with ROS2 commands

echo "=================================================="
echo "SCOUT MINI QUICK MOVEMENT TEST"
echo "=================================================="
echo "⚠️  WARNING: Robot will move!"
echo "Make sure there's clear space around the robot"
echo "=================================================="

# Source ROS2
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

echo ""
read -p "Press Enter to start tests (Ctrl+C to abort)..."

# Function to send command and wait
send_cmd() {
    local description=$1
    local cmd=$2
    local duration=${3:-2}

    echo ""
    echo "Testing: $description"
    echo "Command: ros2 topic pub $cmd"
    echo "Duration: ${duration}s"

    # Send command in background
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "$cmd" &

    # Hold for duration
    sleep $duration

    # Stop robot
    echo "Stopping..."
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
    sleep 1
}

echo ""
echo "=================================================="
echo "STARTING MOVEMENT TESTS"
echo "=================================================="

# Test 1: Forward
send_cmd "FORWARD (0.3 m/s)" "{linear: {x: 0.3}}" 2

# Test 2: Backward
send_cmd "BACKWARD (-0.3 m/s)" "{linear: {x: -0.3}}" 2

# Test 3: Rotate Left (CCW)
send_cmd "ROTATE LEFT (0.5 rad/s)" "{angular: {z: 0.5}}" 2

# Test 4: Rotate Right (CW)
send_cmd "ROTATE RIGHT (-0.5 rad/s)" "{angular: {z: -0.5}}" 2

# Test 5: Arc Left
send_cmd "ARC LEFT (forward + left)" "{linear: {x: 0.3}, angular: {z: 0.3}}" 3

# Test 6: Arc Right
send_cmd "ARC RIGHT (forward + right)" "{linear: {x: 0.3}, angular: {z: -0.3}}" 3

# Test 7: Speed Test
echo ""
echo "=================================================="
echo "SPEED TEST - Progressive speeds"
echo "=================================================="

for speed in 0.1 0.2 0.3 0.4 0.5; do
    send_cmd "Speed: $speed m/s" "{linear: {x: $speed}}" 1
done

# Final stop
echo ""
echo "Final STOP"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"

echo ""
echo "=================================================="
echo "✅ ALL TESTS COMPLETED!"
echo "=================================================="
echo ""
echo "Test Summary:"
echo "✓ Forward/Backward movement"
echo "✓ Left/Right rotation"
echo "✓ Arc movements"
echo "✓ Speed variations"
echo ""
echo "To test manually, use:"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo "=================================================="