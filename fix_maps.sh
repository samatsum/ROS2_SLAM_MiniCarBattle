#!/bin/bash
set -e

# Map Fix Script for F1TENTH
# Run this on your Notebook PC

echo "=========================================="
echo "Applying Map & Config Fixes"
echo "=========================================="

# 1. Update Repository
echo "[1/3] Updating repository..."
cd ~/f1tenth_ws/src/ROS2_SLAM_MiniCarBattle
git pull origin main

# 2. Check for Overrides
echo "[2/3] Checking for config files..."
OVERRIDES_DIR="src/ROS2_SLAM_MiniCarBattle/config_overrides"

if [ ! -d "$OVERRIDES_DIR" ]; then
    echo "ERROR: config_overrides folder not found at $OVERRIDES_DIR!"
    echo "Current directory: $(pwd)"
    ls -R
    exit 1
fi

# 3. Apply Overrides
echo "[3/3] Copying files..."
# Move up to workspace src
cd ../..
# Copy from the deep nested path
cp -v src/ROS2_SLAM_MiniCarBattle/$OVERRIDES_DIR/gym_bridge_launch.py src/f1tenth_gym_ros/launch/
cp -v src/ROS2_SLAM_MiniCarBattle/$OVERRIDES_DIR/sim.yaml src/f1tenth_gym_ros/config/
cp -v src/ROS2_SLAM_MiniCarBattle/$OVERRIDES_DIR/CourseImage_Standard.* src/f1tenth_gym_ros/maps/

echo "=========================================="
echo "Fix Complete! Please try launching again."
echo "=========================================="
