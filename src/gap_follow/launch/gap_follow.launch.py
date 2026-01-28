from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # --- 設定: リスポーン位置 (ここを変えればすぐ反映されます) ---
    spawn_x_arg = DeclareLaunchArgument("spawn_x", default_value="5.0")
    spawn_y_arg = DeclareLaunchArgument("spawn_y", default_value="1.0")
    spawn_theta_arg = DeclareLaunchArgument("spawn_theta", default_value="3.14")

    # 1. 軌跡ロガーノード
    logger_node = Node(
        package="gap_follow",
        executable="data_logger.py",
        name="trajectory_logger",
        output="screen",
        parameters=[
            {
                "map_resolution": 0.01,
                "map_origin_x": 0.0,
                "map_origin_y": 0.0,
            }
        ],
    )

    # 2. 走行ノード (Gap Follow)
    gap_follow_node = Node(
        package="gap_follow",
        executable="reactive_node",
        name="reactive_node",
        output="screen",
        parameters=[
            {"velocity": 2.5},
            {"target_dist": 0.8},
            {"kp": 1.5},
        ],
    )

    # 3. リセット管理ノード (Sim Reset)
    sim_reset_node = Node(
        package="gap_follow",
        executable="sim_reset_node",
        name="sim_reset_node",
        output="screen",
        parameters=[
            {
                "spawn_x": LaunchConfiguration("spawn_x"),
                "spawn_y": LaunchConfiguration("spawn_y"),
                "spawn_theta": LaunchConfiguration("spawn_theta"),
                "crash_dist": 0.25,
            }
        ],
    )

    return LaunchDescription(
        [
            spawn_x_arg,
            spawn_y_arg,
            spawn_theta_arg,
            logger_node,      # 1. ログ記録
            gap_follow_node,  # 2. 車を動かす
            sim_reset_node,   # 3. リセット管理
        ]
    )

