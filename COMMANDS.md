# 🏎️ Command Cheat Sheet

[日本語 (Japanese)](COMMANDS_ja.md)

Useful commands for daily development after setup.

## 1. Running the Simulation

You need **two terminals**.

**Terminal 1 (Simulator):**
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

**Terminal 2 (Autonomous Driver):**
```bash
ros2 launch gap_follow gap_follow.launch.py
```

## 2. Rebuild After Changes

When you modify `gap_follow.launch.py` or `.cpp` files:

```bash
cd ~/f1tenth_ws
colcon build --packages-select gap_follow --symlink-install
source install/setup.bash
```

*   `--packages-select gap_follow`: Builds only your package (faster).
*   `--symlink-install`: Reflects Python changes immediately without rebuild (useful to have).

## 3. Checking Logs

To open the log directory in your file manager:

```bash
xdg-open ~/f1tenth_ws/log/courseLog/
```

To view the latest log images immediately:

```bash
cd ~/f1tenth_ws/log/courseLog/$(ls -t ~/f1tenth_ws/log/courseLog/ | head -n 1)/images
eog .
```

## 4. Troubleshooting

### Simulator Frozen / Glitched
Kill all ROS 2 processes:

```bash
pkill -f ros
```

Then restart both launch commands in separate terminals.

### Check Running Nodes

```bash
ros2 node list
```

### Check Running Topics

```bash
ros2 topic list
```

### view Topic Data (e.g., LiDAR)

```bash
ros2 topic echo /scan
```
(Press `Ctrl + C` to stop)
