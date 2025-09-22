# Quick Start Guide - Complete Robot System

## Full System Startup (After Restart)

### Option 1: Start Everything Automatically (NEW - RECOMMENDED!)
```bash
~/robotics_projects/lidar_tools/start_all.sh
```
✨ This launches ALL systems in separate terminals:
- Robot base (movement control)
- LiDAR sensor
- Camera sensor
- RViz2 visualization

### Option 2: Manual Step-by-Step Launch:

#### Terminal 1: Start Robot Base (Movement)
```bash
~/robotics_projects/lidar_tools/start_robot_base.sh
```
Wait for: "Scout command node ready!"

#### Terminal 2: Start LiDAR
```bash
~/robotics_projects/lidar_tools/start_lidar.sh
```
Wait for: "RoboSense-LiDAR-Driver is running....."

#### Terminal 3: Start Camera
```bash
~/robotics_projects/lidar_tools/start_realsense.sh
```
Wait for: "RealSense Node Is Up!"

#### Terminal 4: Start Visualization
```bash
~/robotics_projects/lidar_tools/start_visualization.sh
```
This script automatically:
- Sets up coordinate frame transforms (map → rslidar)
- Launches RViz2 with your saved config

## Important Notes for RViz2:

### ✅ Your saved config should remember:
- Fixed Frame: `rslidar`
- PointCloud2 display for `/rslidar_points`
- Image display for `/camera/camera/color/image_raw`

### ⚠️ If RViz shows "Frame [rslidar] does not exist":
The visualization script handles this automatically now, but if needed manually:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map rslidar
```

## Quick Health Check:
```bash
# Check all topics are publishing
ros2 topic list | grep -E "(rslidar|camera)"

# Check LiDAR data rate (should be ~10 Hz)
ros2 topic hz /rslidar_points

# Check camera data rate (should be ~10-30 Hz)
ros2 topic hz /camera/camera/color/image_raw
```

## Troubleshooting:

### Nothing in RViz?
1. Check Fixed Frame is set to `rslidar` (not `map`)
2. Make sure sensors are running (check terminals 1 & 2)
3. Restart visualization script

### Camera image not showing?
- Look for a separate image window (might be minimized)
- Check Reliability Policy is set to "Best Effort" in Image display settings

### Point cloud not visible?
- Adjust PointCloud2 → Size to 0.01 or 0.02
- Check Style is set to "Points" or "Squares"

## Complete Shutdown:
Simply close each terminal window (Ctrl+C) in reverse order:
1. First close RViz2
2. Then stop camera
3. Finally stop LiDAR

---
*All scripts are in: ~/robotics_projects/lidar_tools/*