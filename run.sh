#!/usr/bin/env bash

# Move to workspace directory
WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$WORKSPACE_DIR"

HOTSPOT_SSID="laptop_flipper"
HOTSPOT_PASS="mafiosu123"

# 1. Clean up any stale simulator/ROS/Docker processes from previous runs
echo "==> Cleaning up background processes..."
pkill -9 -f gz 2>/dev/null
pkill -9 -f ros2 2>/dev/null
pkill -9 -f rviz2 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null
pkill -9 -f ros_gz_bridge 2>/dev/null
pkill -9 -f esp32_controller 2>/dev/null
pkill -9 -f teleop_keyboard 2>/dev/null
pkill -9 -f trajectory_tracker 2>/dev/null

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

# 3. Stop and kill previous micro-ROS agent container
echo "==> Restarting micro-ROS Docker Agent..."
docker stop microros_agent 2>/dev/null || true
docker rm -f microros_agent 2>/dev/null || true

# 4. Launch micro-ROS agent in detached mode
docker run -d --rm --net=host --name microros_agent microros/micro-ros-agent:jazzy udp4 --port 8888

# 5. Source ROS 2 Jazzy underlay
echo "==> Sourcing ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash

# 6. Build workspace
echo "==> Building Workspace..."
colcon build --symlink-install

# 7. Source workspace overlay
echo "==> Sourcing Workspace Overlay..."
source install/setup.bash

# 8. Open a new terminal window for Multi-Key Teleop
echo "==> Launching Multi-Key Teleop Window..."
gnome-terminal --title="Flipper Car TC1508 Teleop" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source '$WORKSPACE_DIR/install/setup.bash'
  echo 'Waiting for simulation & bridge to initialize...'
  sleep 4
  ros2 run flipper_car teleop_keyboard
  exec bash
"

# 9. Launch Gazebo, Bridges, Controller, Tracker, and RViz
echo "==> Starting Simulation, Bridge, Controller, Tracker & RViz..."
ros2 launch flipper_car flipper_car.launch.py