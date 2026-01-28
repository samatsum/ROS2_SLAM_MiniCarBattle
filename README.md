# ROS 2 F1TENTH Autonomous Driving Starter Kit (Beginner's Guide)

[日本語 (Japanese)](README_ja.md)

This repository implements the **Follow the Gap** algorithm for the F1TENTH autonomous racing simulator. The car navigates autonomously while avoiding obstacles using LiDAR data.

This guide is designed for **absolute beginners** who have never used ROS 2 before. Just follow the steps below!

---

## 🏎️ What does this do?

*   Autonomously drives an F1 car in a virtual simulator.
*   Uses **LiDAR** sensor data to steer away from walls.
*   Saves the driving trajectory as an image.

---

## 💻 1. Prerequisites (Recommended Environment)

You need the **Ubuntu** operating system to run this program.

*   **PC**: Standard Laptop or Desktop
*   **OS**: **Ubuntu 22.04 LTS (Jammy Jellyfish)**
    *   *Note: Windows/Mac users should use WSL2 or VirtualBox to set up Ubuntu 22.04.*
*   **Python**: Version **3.10** (Standard in Ubuntu 22.04)
    *   *Note: Other versions may cause compatibility issues.*

---

## 🛠️ 2. Installation Guide

Open your terminal (`Ctrl + Alt + T`) and copy-paste the following commands one block at a time.

### Step 1: Install ROS 2 (Humble)

Install the core software "ROS 2 Humble".

```bash
# 1. Set Locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Add Repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# 3. Add ROS 2 Key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 4. Add Repository to Source List
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 5. Install ROS 2 (This takes a while ☕)
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop -y

# 6. Setup Environment Variables
grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc || echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 7. Install Build Tools
sudo apt install python3-colcon-common-extensions python3-rosdep -y
```

### Step 2: Setup Workspace & Download Simulator

Create a workspace folder and download the necessary code.

```bash
# 1. Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# 2. Create Workspace
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws/src

# 3. Download F1TENTH Simulator
git clone https://github.com/f1tenth/f1tenth_gym_ros.git

# 4. Download This Autonomous Driving Code
git clone https://github.com/samatsum/ROS2_SLAM_MiniCarBattle.git
```

### Step 3: Install Dependencies & Build

Install required libraries and build the program.

```bash
# 1. Go to Workspace Root
cd ~/f1tenth_ws

# 2. Install Dependencies
rosdep install -i --from-path src --rosdistro humble -y

# 3. Build (This also takes a while ☕)
colcon build

# 4. Source the Workspace
source install/setup.bash
echo "source ~/f1tenth_ws/install/setup.bash" >> ~/.bashrc
```

---

## 🚀 3. How to Run

To run the simulation, you need **two separate terminals**.

### Terminal 1: Launch the Simulator

Open a new terminal and run:

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
> **What generally happens:** An RViz window will open showing the track and car.

### Terminal 2: Start Autonomous Driving

Open **another** terminal and run:

```bash
ros2 launch gap_follow gap_follow.launch.py
```
> **What should happen:** The car will start moving automatically around the track! 🏎️💨

### checking Logs

After driving (Strop with `Ctrl + C`), logs and images are saved in:

`~/f1tenth_ws/log/courseLog/YYYYMMDD_HHMMSS_manual/images/`

You will find `kinematics.png` (driving path) and map images here.

---

## ⚙️ 4. Configuration

To change speed or driving parameters, edit this file:

`~/f1tenth_ws/src/ROS2_SLAM_MiniCarBattle/src/gap_follow/launch/gap_follow.launch.py`

```python
        parameters=[
            {"velocity": 2.5},      # Speed (m/s)
            {"target_dist": 0.8},   # Desired distance to wall
            {"kp": 1.5},            # Steering sensitivity
        ],
```

After editing, rebuild the package:
```bash
cd ~/f1tenth_ws
colcon build --packages-select gap_follow --symlink-install
source install/setup.bash
```

---

## ❓ Troubleshooting

**Q. `ros2: command not found`**
A. Run `source ~/.bashrc` or restart your terminal.

**Q. Build failed**
A. Did you skip `rosdep install ...`? You might be missing dependencies.

**Q. Car doesn't move**
A. Make sure **BOTH** terminals are running. Try pressing "Reset" in RViz, or restart both launch commands.
