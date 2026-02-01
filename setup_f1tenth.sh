#!/bin/bash
set -e

# F1TENTH Workspace Setup Script
# Run this on your Notebook PC (Ubuntu 22.04 + ROS 2 Humble)

echo "=========================================="
echo "Starting F1TENTH Workspace Setup"
echo "=========================================="

# 1. Install Build Tools
echo "[1/6] Installing build tools (rosdep, colcon)..."
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-pip git

# 2. Initialize rosdep
echo "[2/6] Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init || echo "rosdep already initialized"
fi
rosdep update

# 3. Create Workspace
echo "[3/6] Creating workspace at ~/f1tenth_ws..."
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws/src

# 4. Clone Repositories
echo "[4/6] Cloning repositories..."

# F1TENTH Simulator
if [ ! -d "f1tenth_gym_ros" ]; then
    git clone https://github.com/f1tenth/f1tenth_gym_ros.git
else
    echo "  - f1tenth_gym_ros already exists."
fi

# Your Autonomous Driving Code
if [ ! -d "ROS2_SLAM_MiniCarBattle" ]; then
    git clone https://github.com/samatsum/ROS2_SLAM_MiniCarBattle.git
else
    echo "  - ROS2_SLAM_MiniCarBattle already exists. Updating..."
    cd ROS2_SLAM_MiniCarBattle
    git pull
    cd ..
fi

# Apply Config Overrides (Fix for map path issues on different usernames)
echo "    -> Applying config overrides..."
OVERRIDES_DIR="ROS2_SLAM_MiniCarBattle/src/ROS2_SLAM_MiniCarBattle/config_overrides"

if [ -d "$OVERRIDES_DIR" ]; then
    cp -v $OVERRIDES_DIR/gym_bridge_launch.py f1tenth_gym_ros/launch/
    cp -v $OVERRIDES_DIR/sim.yaml f1tenth_gym_ros/config/
    cp -v $OVERRIDES_DIR/CourseImage_Standard.* f1tenth_gym_ros/maps/
    echo "    -> Overrides and maps applied successfully."
else
    echo "    -> No overrides found at $OVERRIDES_DIR"
fi

# 5. Install Dependencies
echo "[5/6] Installing dependencies..."
cd ~/f1tenth_ws
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y

# 6. Build Workspace
echo "[6/6] Building workspace (this may take a few minutes)..."
colcon build

# 7. Add to .bashrc
if ! grep -q "source ~/f1tenth_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/f1tenth_ws/install/setup.bash" >> ~/.bashrc
    echo "  - Added workspace setup to ~/.bashrc"
else
    echo "  - ~/.bashrc already configured."
fi

echo "=========================================="
echo "Setup Complete!"
echo "Please close and reopen your terminal, or run: source ~/.bashrc"
echo "=========================================="
