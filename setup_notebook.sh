#!/bin/bash
# ノートPC用 F1TENTH環境セットアップスクリプト
# このスクリプトをclone後に実行してください

set -e

echo "=============================================="
echo "F1TENTH環境セットアップスクリプト (Ubuntu 22.04 / ROS2 Humble)"
echo "=============================================="

# ROS2 Humbleのインストール確認
echo ""
echo "[Step 1/6] ROS2 Humbleのインストール確認..."
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "    ROS2 Humble: インストール済み ✓"
    source /opt/ros/humble/setup.bash
else
    echo "    ROS2 Humbleがインストールされていません。"
    echo "    以下のURLを参照してインストールしてください:"
    echo "    https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html"
    exit 1
fi

# システム依存関係のインストール
echo ""
echo "[Step 2/6] システム依存関係をインストール中..."
sudo apt-get update
sudo apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-rviz2 \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-ackermann-msgs \
    ros-humble-teleop-twist-keyboard \
    ros-humble-tf2-ros \
    libeigen3-dev

# Python依存関係
echo ""
echo "[Step 3/6] Python依存関係をインストール中..."
pip3 install transforms3d
pip3 install numpy
pip3 install pyyaml
pip3 install matplotlib
pip3 install pandas

# f1tenth_gymのインストール
echo ""
echo "[Step 4/6] f1tenth_gymをインストール中..."
if [ ! -d "$HOME/f1tenth_gym" ]; then
    cd $HOME
    git clone https://github.com/f1tenth/f1tenth_gym.git
    cd f1tenth_gym
    pip3 install -e .
else
    echo "    f1tenth_gym: 既にインストール済み ✓"
    cd $HOME/f1tenth_gym
    pip3 install -e . --quiet
fi

# rosdepの初期化と依存関係のインストール
echo ""
echo "[Step 5/6] ROS依存関係を解決中..."
cd ~/f1tenth_ws
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init || true
fi
rosdep update || true
rosdep install -i --from-path src --rosdistro humble -y || true

# ワークスペースのビルド
echo ""
echo "[Step 6/6] ワークスペースをビルド中..."
cd ~/f1tenth_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

echo ""
echo "=============================================="
echo "セットアップ完了！"
echo "=============================================="
echo ""
echo "使用方法:"
echo "  1. 新しいターミナルを開く"
echo "  2. source ~/f1tenth_ws/install/setup.bash"
echo "  3. ros2 launch f1tenth_gym_ros gym_bridge_launch.py"
echo ""
echo "~/.bashrcに以下を追加することをお勧めします:"
echo '  source /opt/ros/humble/setup.bash'
echo '  source ~/f1tenth_ws/install/setup.bash'
echo ""
