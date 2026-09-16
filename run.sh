#!/usr/bin/env bash
set -e

# Move to workspace directory
WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$WORKSPACE_DIR"

HOTSPOT_SSID="laptop_flipper"
HOTSPOT_PASS="mafiosu123"

# 1. Clean up background processes
echo "==> Cleaning up background processes..."
pkill -9 -f gz 2>/dev/null || true
pkill -9 -f ros2 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f ros_gz_bridge 2>/dev/null || true
pkill -9 -f teleop_keyboard 2>/dev/null || true
pkill -9 -f trajectory_tracker 2>/dev/null || true

# 2. Ensure Laptop Wi-Fi Hotspot is Active
echo "==> Checking and activating Wi-Fi Hotspot ($HOTSPOT_SSID)..."
WIFI_DEV=$(nmcli device status | awk '$2=="wifi" {print $1; exit}')

if [ -z "$WIFI_DEV" ]; then
    echo "⚠️ Warning: No Wi-Fi device found. Skipping hotspot activation."
else
    # Check if the connection profile already exists
    if nmcli connection show "$HOTSPOT_SSID" >/dev/null 2>&1; then
        nmcli connection up "$HOTSPOT_SSID" || true
    else
        # Create and launch hotspot if profile is missing
        nmcli device wifi hotspot ifname "$WIFI_DEV" ssid "$HOTSPOT_SSID" password "$HOTSPOT_PASS" || true
    fi
fi

# 3. Restart micro-ROS Docker Agent
echo "==> Restarting micro-ROS Docker Agent..."
docker stop microros_agent 2>/dev/null || true
docker rm -f microros_agent 2>/dev/null || true
docker run -d --rm --net=host --name microros_agent microros/micro-ros-agent:jazzy udp4 --port 8888

# 4. Source ROS 2
echo "==> Sourcing ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash

# 5. Build Workspace
echo "==> Building Workspace..."
colcon build --symlink-install --packages-select bulldozer micro_ros_msgs

# 6. Source Workspace Overlay
echo "==> Sourcing Workspace Overlay..."
source install/setup.bash

# 7. Set Gazebo Mesh Resource Path
export GZ_SIM_RESOURCE_PATH="$WORKSPACE_DIR/install/bulldozer/share:$WORKSPACE_DIR/src/bulldozer:$GZ_SIM_RESOURCE_PATH"

# 8. Launch Teleop Terminal
echo "==> Launching Bulldozer Multi-Key Teleop Window..."
gnome-terminal --title="Bulldozer Claw Teleop" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source '$WORKSPACE_DIR/install/setup.bash'
  echo 'Waiting for simulation & micro-ROS bridge to initialize...'
  sleep 3
  ros2 run bulldozer teleop_keyboard
  exec bash
"

# 9. Launch Gazebo, Bridges, State Publisher, Tracker, and RViz
echo "==> Starting Simulation, Bridge, Tracker & RViz..."
ros2 launch bulldozer bulldozer.launch.py