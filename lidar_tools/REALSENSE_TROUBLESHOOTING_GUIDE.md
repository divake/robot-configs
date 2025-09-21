# RealSense D435 Camera Troubleshooting Guide

## Camera Information
- **Model:** Intel RealSense D435
- **Serial Number:** 317622072348
- **USB Type:** USB 2.1 (reduced performance warning is normal)
- **Physical Port:** 3-7.1.1

## Quick Verification After Restart

### 1. Check USB Connection
```bash
# Check if camera is detected
lsusb | grep "Intel Corp. RealSense"
```
**Expected output:** `Bus 003 Device XXX: ID 8086:0b07 Intel Corp. RealSense D435`

### 2. Check RealSense Viewer (Optional)
```bash
# Test camera with Intel's viewer
/usr/local/bin/realsense-viewer
```
This should show live RGB and depth streams if camera is working.

### 3. Launch ROS2 Camera Node
```bash
~/robotics_projects/lidar_tools/start_realsense.sh
```
**Look for:** "RealSense Node Is Up!"

### 4. Verify ROS2 Topics
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep camera
```
**Expected topics:**
- `/camera/camera/color/image_raw` - RGB stream
- `/camera/camera/depth/image_rect_raw` - Depth stream
- `/camera/camera/color/camera_info` - Camera calibration

### 5. Check Data Flow
```bash
# Check RGB stream rate (should be ~10-30 Hz)
ros2 topic hz /camera/camera/color/image_raw

# Check depth stream rate
ros2 topic hz /camera/camera/depth/image_rect_raw
```

## Common Issues and Solutions

### Issue 1: Camera Not Detected
**Symptom:** `lsusb` doesn't show RealSense device
**Solutions:**
1. Check USB cable connection
2. Try different USB port (prefer USB 3.0 for better performance)
3. Unplug and replug the camera
4. Check power: `dmesg | tail -20` after plugging in

### Issue 2: Permission Denied
**Symptom:** Cannot access camera, permission errors
**Fix:** Add udev rules
```bash
# Download and install RealSense udev rules
wget https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
sudo mv 99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# Logout and login again
```

### Issue 3: No ROS2 Topics Published
**Symptom:** Camera launches but no topics appear
**Checks:**
1. Kill existing processes: `pkill -f realsense`
2. Source ROS2: `source /opt/ros/jazzy/setup.bash`
3. Restart the node: `~/robotics_projects/lidar_tools/start_realsense.sh`

### Issue 4: Low Frame Rate Warning
**Symptom:** "Device is connected using a 2.1 port. Reduced performance is expected"
**Note:** This is normal if connected to USB 2.1 port. For better performance:
- Use USB 3.0 port (blue port) if available
- Current setup works fine at 10-15 FPS

### Issue 5: Camera Firmware Update
**Symptom:** Old firmware version warnings
**Current version:** 5.17.0.10
**To update (if needed):**
```bash
# Use Intel's firmware update tool
/usr/local/bin/rs-fw-update -l  # List devices
/usr/local/bin/rs-fw-update -f firmware.bin  # Update (get firmware from Intel website)
```

### Issue 6: Multiple Camera Processes
**Symptom:** "Device or resource busy" errors
**Fix:**
```bash
# Find and kill all RealSense processes
ps aux | grep realsense
pkill -f realsense
pkill -f rs_launch
```

## Camera Configuration Options

### Basic Launch
```bash
ros2 launch realsense2_camera rs_launch.py
```

### With Custom Settings
```bash
ros2 launch realsense2_camera rs_launch.py \
    depth_module.depth_profile:=640x480x30 \
    rgb_camera.color_profile:=640x480x30 \
    enable_sync:=true \
    align_depth.enable:=true
```

### Available Profiles
- **Color:** 640x480x30, 1280x720x30, 1920x1080x30
- **Depth:** 640x480x30, 848x480x30, 1280x720x30
- **Lower resolution = Higher FPS**

## Useful Commands

### View Raw Image (using ROS2)
```bash
# View RGB image
ros2 run rqt_image_view rqt_image_view
# Select: /camera/camera/color/image_raw

# View depth as image
ros2 run rqt_image_view rqt_image_view
# Select: /camera/camera/depth/image_rect_raw
```

### Record Camera Data
```bash
# Record to ROS2 bag
ros2 bag record /camera/camera/color/image_raw /camera/camera/depth/image_rect_raw
```

### Camera Info and Calibration
```bash
# View camera calibration parameters
ros2 topic echo /camera/camera/color/camera_info --once
```

## System Architecture
```
RealSense D435 (USB 2.1/3.0)
    |
    | LibRealSense SDK
    ↓
ROS2 RealSense Node
    |
    | ROS2 Topics
    ↓
├── /camera/camera/color/image_raw (RGB)
├── /camera/camera/depth/image_rect_raw (Depth)
├── /camera/camera/color/camera_info (Calibration)
└── /camera/camera/depth/camera_info (Calibration)
```

## Important Files and Locations
- **Launch script:** `~/robotics_projects/lidar_tools/start_realsense.sh`
- **ROS2 package:** `/opt/ros/jazzy/share/realsense2_camera/`
- **Config files:** `/opt/ros/jazzy/share/realsense2_camera/config/`
- **RealSense viewer:** `/usr/local/bin/realsense-viewer`

## Quick Health Check (One-Liner)
```bash
echo "USB: $(lsusb | grep -q 'RealSense D435' && echo 'OK' || echo 'FAIL')" && \
echo "ROS2 Node: $(ros2 node list | grep -q 'camera' && echo 'OK' || echo 'FAIL')" && \
echo "Topics: $(ros2 topic list | grep -q '/camera/camera/color' && echo 'OK' || echo 'FAIL')"
```

## Integration with LiDAR
Both sensors can run simultaneously:
```bash
# Terminal 1: LiDAR
~/robotics_projects/lidar_tools/start_lidar.sh

# Terminal 2: Camera
~/robotics_projects/lidar_tools/start_realsense.sh

# Terminal 3: View both in RViz2
rviz2
# Add PointCloud2 display for /rslidar_points
# Add Image display for /camera/camera/color/image_raw
```

## Performance Notes
- Camera runs at ~10-15 FPS on USB 2.1
- Both camera and LiDAR together use moderate CPU (~20-30%)
- No conflicts between devices (different interfaces: Ethernet vs USB)

---
*Created: September 21, 2025*
*Camera Model: Intel RealSense D435*
*ROS2 Version: Jazzy*
*Host: Intel NUC with Ubuntu*