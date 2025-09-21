# Git Version Control Strategy for Robotics Projects

## Recommended Repository Structure

### Repository 1: `robot-configs` (Your Custom Configurations)
**Location:** `~/robotics_projects/`

**TRACK these files:**
```
robotics_projects/
├── lidar_tools/
│   ├── *.sh (launch scripts)
│   ├── *.md (documentation)
│   └── config files
├── scout_ros2/
│   └── custom launch files
└── README.md
```

**Git repository for:**
- All troubleshooting guides
- Launch scripts
- Configuration files
- Documentation
- Custom tools

### Repository 2: `ros2-custom-packages` (Your ROS2 Development)
**Location:** `~/ros2_ws/src/`

**TRACK only YOUR packages:**
```
ros2_ws/src/
├── my_robot_control/     # Your custom packages
├── my_slam_package/       # Your developments
└── .gitignore            # Ignore vendor packages
```

**DO NOT track vendor packages:**
- rslidar_sdk (vendor - clone separately)
- realsense-ros (vendor - clone separately)
- scout_ros2 (vendor - clone separately)

## What to Track vs Ignore

### ✅ ALWAYS Track
- Custom Python/C++ source code
- Launch files you create
- Configuration YAML files you modify
- Documentation (*.md)
- Shell scripts
- Package.xml and CMakeLists.txt (for your packages)
- Custom message definitions

### ❌ NEVER Track
- Build directories (`build/`, `install/`, `log/`)
- Binary files (*.so, *.pyc)
- ROS bags (too large, store elsewhere)
- Vendor packages (document versions instead)
- IDE files (.vscode/, .idea/)
- System files (.DS_Store)
- Credentials or IP addresses (use environment variables)

## Setting Up Git

### 1. For robotics_projects
```bash
cd ~/robotics_projects
git init
cat > .gitignore << 'EOF'
# Build artifacts
build/
install/
log/
*.pyc
__pycache__/

# ROS specific
*.bag
*.db3
.ros/

# System
.DS_Store
*.swp
*~

# Credentials (if any)
.env
secrets/
EOF

git add lidar_tools/
git commit -m "Initial commit: LiDAR and camera tools with documentation"
```

### 2. For ros2_ws (only your packages)
```bash
cd ~/ros2_ws/src
git init
cat > .gitignore << 'EOF'
# Vendor packages (clone these separately)
rslidar_sdk/
realsense-ros/
scout_ros2/
ugv_sdk/

# Build artifacts
build/
install/
log/

# ROS2
*.bag
*.db3

# Python
*.pyc
__pycache__/
EOF

# Add only YOUR custom packages when you create them
git add my_package/  # Example
git commit -m "Initial commit: Custom ROS2 packages"
```

## Vendor Package Management

Create a `DEPENDENCIES.md` file:
```markdown
# External Dependencies

## ROS2 Packages
- rslidar_sdk: https://github.com/RoboSense-LiDAR/rslidar_sdk.git (branch: ros2)
- realsense-ros: v4.56.4 (installed via apt)

## System Packages
- ROS2 Jazzy
- librealsense2: 2.56.4
- Ubuntu 24.04 LTS
```

## GitHub Repository Setup

### Create Two Repos:
1. **robot-configs**
   - Description: "Configuration, scripts, and documentation for robot sensors"
   - Make it PUBLIC (helps others with similar hardware)

2. **my-ros2-packages** (when you start developing)
   - Description: "Custom ROS2 packages for robot control"
   - Can be PRIVATE initially

### Push to GitHub:
```bash
# For robotics_projects
cd ~/robotics_projects
git remote add origin https://github.com/YOUR_USERNAME/robot-configs.git
git branch -M main
git push -u origin main
```

## Backup Strategy

### Critical Files to Version Control:
1. **Network configuration** (static IP setup)
2. **Modified config files** (rslidar config.yaml)
3. **Launch scripts** (start_lidar.sh, start_realsense.sh)
4. **Troubleshooting guides** (all .md files)

### Use Git Tags for Working Configurations:
```bash
git tag -a "v1.0-working-lidar-camera" -m "Both sensors working"
git push origin --tags
```

## Recovery Plan
If something breaks, you can always:
```bash
git checkout v1.0-working-lidar-camera
```

## Best Practices
1. Commit after each successful configuration
2. Write descriptive commit messages
3. Tag known-good configurations
4. Document IP addresses and network setup in README
5. Use branches for experimental changes
6. Regular pushes to GitHub (daily backup)

---
*This strategy keeps your custom work safe while avoiding bloat from vendor code*