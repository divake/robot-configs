# RSLidar Troubleshooting Guide

## What Was Fixed (Permanent Solution)

### Root Problem
Your host computer was getting a **dynamic IP address** from DHCP (changing every restart: 192.168.1.190, 192.168.1.185, etc.), but the LiDAR was configured to send data to a fixed IP (192.168.1.244).

### Permanent Fix Applied
1. **Set static IP on host**: 192.168.1.244 (matches what LiDAR expects)
   ```bash
   sudo nmcli connection modify netplan-enp86s0 ipv4.addresses 192.168.1.244/24 ipv4.gateway 192.168.1.1 ipv4.dns "8.8.8.8 8.8.4.4" ipv4.method manual
   sudo nmcli connection down netplan-enp86s0 && sudo nmcli connection up netplan-enp86s0
   ```

2. **Updated RSLidar config** to match: `/home/nus-ai/ros2_ws/src/rslidar_sdk/config/config.yaml`
   - host_address: 192.168.1.244
   - lidar_address: 192.168.1.200

## Step-by-Step Verification After Restart

### 1. Check Your Network Interface and IP
```bash
# Check your network interface name (should be enp86s0)
ip link show

# Check your IP address (MUST be 192.168.1.244)
ip addr show enp86s0 | grep inet
```
**Expected output:** `inet 192.168.1.244/24`

### 2. Check LiDAR Physical Connection
- LiDAR power light should be ON
- LAN cable connected and blinking
- LiDAR should be warm to touch (indicates it's running)

### 3. Ping the LiDAR
```bash
ping -c 3 192.168.1.200
```
**Expected:** Should get replies with low latency (~1ms)

### 4. Check RSLidar Configuration
```bash
grep host_address ~/ros2_ws/src/rslidar_sdk/config/config.yaml
```
**Expected output:** `host_address: 192.168.1.244`

### 5. Launch the LiDAR Driver
```bash
~/robotics_projects/lidar_tools/start_lidar.sh
```
**Look for:** "RoboSense-LiDAR-Driver is running....."

### 6. Verify Data is Being Published
Open a new terminal and run:
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep rslidar
```
**Expected output:**
- /rslidar_packets
- /rslidar_points

### 7. Check if Point Cloud Data is Coming
```bash
ros2 topic hz /rslidar_points
```
**Expected:** Should show ~10Hz rate

### 8. View Actual Data (Optional)
```bash
ros2 topic echo /rslidar_points --once | head -20
```
**Expected:** Should show point cloud header with data

## Common Issues and Solutions

### Issue 1: IP Address Changed After Restart
**Symptom:** `ip addr show` shows different IP than 192.168.1.244
**Fix:** Re-apply static IP configuration
```bash
sudo nmcli connection modify netplan-enp86s0 ipv4.addresses 192.168.1.244/24 ipv4.method manual
sudo nmcli connection down netplan-enp86s0 && sudo nmcli connection up netplan-enp86s0
```

### Issue 2: Can't Ping LiDAR
**Symptom:** `ping 192.168.1.200` times out
**Checks:**
1. Physical cable connection
2. LiDAR power
3. Correct network interface: `ip route | grep 192.168.1.0`

### Issue 3: MSOPTIMEOUT Errors
**Symptom:** Driver shows "ERRCODE_MSOPTIMEOUT"
**Meaning:** LiDAR isn't sending data to your host
**Fix:**
1. Check your IP is 192.168.1.244
2. Access LiDAR web interface at http://192.168.1.200
3. Set destination IP to 192.168.1.244 in LiDAR settings

### Issue 4: No ROS2 Topics
**Symptom:** `ros2 topic list` doesn't show /rslidar_points
**Fix:**
1. Kill any existing processes: `pkill -f rslidar`
2. Restart the driver: `~/robotics_projects/lidar_tools/start_lidar.sh`

## Network Architecture
```
LiDAR (192.168.1.200)
    |
    | UDP packets on ports 6699 (MSOP) & 7788 (DIFOP)
    ↓
Host PC (192.168.1.244)
    |
    | ROS2 topics
    ↓
/rslidar_points (Point Cloud)
```

## Important Files and Locations
- **Launch script:** `~/robotics_projects/lidar_tools/start_lidar.sh`
- **Config file:** `~/ros2_ws/src/rslidar_sdk/config/config.yaml`
- **ROS2 workspace:** `~/ros2_ws/`
- **Network interface:** `enp86s0`

## Quick Test After Restart
Run this one-liner to check everything:
```bash
echo "IP: $(ip addr show enp86s0 | grep 192.168.1.244 && echo 'OK' || echo 'FAIL')" && \
echo "Ping LiDAR: $(ping -c 1 -W 1 192.168.1.200 > /dev/null 2>&1 && echo 'OK' || echo 'FAIL')" && \
echo "Config: $(grep -q 'host_address: 192.168.1.244' ~/ros2_ws/src/rslidar_sdk/config/config.yaml && echo 'OK' || echo 'FAIL')"
```

## If Everything Fails
Check if packets are actually arriving:
```bash
sudo tcpdump -i enp86s0 -n udp port 6699 -c 5
```
If no packets, the LiDAR needs reconfiguration via its web interface.

---
*Created: September 21, 2025*
*LiDAR Model: RSLidar HELIOS 16P*
*Host: Intel NUC with Ubuntu*