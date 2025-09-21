# Robot Configuration and Tools

This repository contains configuration files, launch scripts, and documentation for a ROS2-based mobile robot equipped with LiDAR and camera sensors.

## Hardware Setup
- **Robot Platform:** Scout Mini
- **LiDAR:** RoboSense RSLidar HELIOS 16P
- **Camera:** Intel RealSense D435
- **Computer:** Intel NUC
- **OS:** Ubuntu 24.04 LTS
- **ROS Version:** ROS2 Jazzy

## Network Configuration
- **Host IP:** 192.168.1.244 (static)
- **LiDAR IP:** 192.168.1.200
- **Interface:** enp86s0

## Directory Structure
```
robotics_projects/
├── lidar_tools/              # Sensor scripts and documentation
│   ├── start_lidar.sh        # Launch RSLidar
│   ├── start_realsense.sh    # Launch RealSense camera
│   ├── LIDAR_TROUBLESHOOTING_GUIDE.md
│   ├── REALSENSE_TROUBLESHOOTING_GUIDE.md
│   └── GIT_STRATEGY.md
└── README.md
```

## Quick Start

### Launch Sensors
```bash
# Terminal 1: Start LiDAR
~/robotics_projects/lidar_tools/start_lidar.sh

# Terminal 2: Start Camera
~/robotics_projects/lidar_tools/start_realsense.sh

# Terminal 3: Visualization
rviz2
```

## ROS2 Topics
- **LiDAR:** `/rslidar_points` - Point cloud data at 10Hz
- **Camera RGB:** `/camera/camera/color/image_raw` - Color images
- **Camera Depth:** `/camera/camera/depth/image_rect_raw` - Depth images

## External Dependencies
- **rslidar_sdk:** https://github.com/RoboSense-LiDAR/rslidar_sdk.git (branch: ros2)
- **realsense-ros:** v4.56.4 (installed via apt)
- **scout_ros2:** AgileX Robotics Scout ROS2 driver
- **ugv_sdk:** AgileX Robotics UGV SDK

## System Packages
```bash
# RealSense
sudo apt install ros-jazzy-realsense2-camera

# Required ROS2 packages
sudo apt install ros-jazzy-diagnostic-updater
sudo apt install ros-jazzy-image-transport-plugins
```

## Troubleshooting
See individual troubleshooting guides in `lidar_tools/`:
- [LiDAR Troubleshooting](lidar_tools/LIDAR_TROUBLESHOOTING_GUIDE.md)
- [Camera Troubleshooting](lidar_tools/REALSENSE_TROUBLESHOOTING_GUIDE.md)

## Known Working Configuration
- **Date:** September 21, 2025
- **Status:** Both LiDAR and camera operational
- **Key Fix:** Set static IP 192.168.1.244 to match LiDAR configuration

## License
Configuration files and scripts for personal robotics research.

## Contact
For questions about this setup, create an issue in this repository.