#!/usr/bin/env bash
set -e

# Move to workspace directory
WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$WORKSPACE_DIR"

# 1. Clean up background processes
echo "==> Cleaning up background processes..."
pkill -9 -f gz 2>/dev/null || true
pkill -9 -f ros2 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f ros_gz_bridge 2>/dev/null || true
pkill -9 -f teleop_keyboard 2>/dev/null || true
pkill -9 -f trajectory_tracker 2>/dev/null || true

# 2. Wi-Fi Hotspot Step Removed
# Both Laptop and ESP32 connect directly to the router ("Project")

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