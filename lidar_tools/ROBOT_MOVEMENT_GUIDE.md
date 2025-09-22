# Scout Mini Robot Movement Guide

## Quick Start After Restart

### Option 1: Start Everything (Recommended)
```bash
~/robotics_projects/lidar_tools/start_all.sh
```
This launches:
- Robot base (movement control)
- LiDAR sensor
- Camera sensor
- RViz2 visualization

### Option 2: Start Only Robot Movement
```bash
~/robotics_projects/lidar_tools/start_robot_base.sh
```

## What Gets Configured

### CAN Interface
- **Interface:** can0
- **Bitrate:** 500000
- **Auto-enabled:** Script checks and enables if needed
- **Manual enable:** `sudo ip link set can0 up type can bitrate 500000`

### ROS2 Node
- **Package:** scout_cmd (your custom stable wrapper)
- **Node:** scout_cmd_node
- **Launch file:** scout_mini.launch.py

## Movement Specifications

### Linear Speed
- **Max Speed:** 1.5 m/s
- **Recommended:** 0.3-0.5 m/s
- **Test Speed:** 0.3 m/s

### Angular Speed
- **Max Speed:** 2.0 rad/s
- **Recommended:** 0.5-1.0 rad/s
- **Test Speed:** 0.5 rad/s

## Control Methods

### 1. Keyboard Teleop
```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
- W/S: Forward/Backward
- A/D: Rotate left/right
- Q/E: Arc left/right
- Space: Stop

### 2. Command Line
```bash
# Forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# Backward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.3}}"

# Rotate left (CCW)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Rotate right (CW)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: -0.5}}"

# Arc motion (forward + turn)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.3}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"
```

### 3. Python Code
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def move_forward(self, speed=0.3):
        msg = Twist()
        msg.linear.x = speed
        self.publisher.publish(msg)

    def rotate(self, angular_speed=0.5):
        msg = Twist()
        msg.angular.z = angular_speed
        self.publisher.publish(msg)

    def stop(self):
        self.publisher.publish(Twist())

# Usage
rclpy.init()
controller = RobotController()
controller.move_forward(0.3)  # Move at 0.3 m/s
```

## Troubleshooting

### CAN Interface Issues

#### Check CAN status
```bash
ip link show can0
```

#### If CAN is DOWN
```bash
sudo ip link set can0 up type can bitrate 500000
```

#### Monitor CAN traffic
```bash
candump can0
```

### Node Not Starting

#### Check if already running
```bash
ros2 node list | grep scout_cmd_node
```

#### Kill existing process
```bash
pkill -f "scout_cmd scout_mini.launch.py"
```

#### Check CAN cable connection
- Ensure CAN adapter is plugged into USB
- Check cable connection to robot
- Verify robot power is ON

### Robot Not Moving

#### Verify cmd_vel is being received
```bash
ros2 topic echo /cmd_vel
```

#### Check odometry for feedback
```bash
ros2 topic echo /odom
```

#### Emergency stop
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"
```

## Testing Movement

### Quick Test
```bash
~/robotics_projects/robot_tests/quick_test.sh
```

### Comprehensive Test
```bash
source /opt/ros/jazzy/setup.bash
python3 ~/robotics_projects/robot_tests/test_scout_movements.py
```

### Interactive Control
```bash
python3 ~/robotics_projects/robot_tests/test_scout_movements.py --interactive
```

## Safety Guidelines

1. **Clear Space:** Ensure 2m radius clear around robot
2. **Emergency Stop:** Always have Ctrl+C ready
3. **Start Slow:** Begin with low speeds (0.1-0.3 m/s)
4. **Monitor Battery:** Low battery affects performance
5. **Check Wheels:** Ensure wheels are free to move

## System Architecture

```
Your Code / Teleop
        ↓
    /cmd_vel
        ↓
scout_cmd_node (Custom wrapper)
        ↓
    ugv_sdk
        ↓
    CAN bus
        ↓
Scout Mini Robot
        ↓
    /odom
```

## Complete Workflow After System Restart

1. **Power on robot** (if not already on)
2. **Run startup script:**
   ```bash
   ~/robotics_projects/lidar_tools/start_robot_base.sh
   ```
   Or for all systems:
   ```bash
   ~/robotics_projects/lidar_tools/start_all.sh
   ```
3. **Verify connection:**
   ```bash
   ros2 node list  # Should show scout_cmd_node
   ros2 topic list # Should show /cmd_vel and /odom
   ```
4. **Test movement:**
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
   sleep 1
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
   ```

## Important Files

- **Startup script:** `~/robotics_projects/lidar_tools/start_robot_base.sh`
- **Complete system:** `~/robotics_projects/lidar_tools/start_all.sh`
- **Test scripts:** `~/robotics_projects/robot_tests/`
- **Custom driver:** `~/scout_mini_ros2_jazzy/`
- **Config:** Uses default CAN settings (no config file needed)

## Notes

- The custom `scout_cmd` wrapper bypasses the unstable official scout_ros2 package
- Directly interfaces with ugv_sdk for stability
- No segmentation faults with this implementation
- Odometry publishes at 50Hz
- Commands accepted at any rate (buffered internally)

## Git Repository

All configurations backed up at:
https://github.com/divake/robot-configs

---
*Last Updated: [Current Date]*
*Tested on: Ubuntu 24.04, ROS2 Jazzy, Scout Mini Pro*