import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from PIL import Image

# ---------------------------------------------------------
# マップの保存場所（srcフォルダの場所を指定）
# ---------------------------------------------------------
MAPS_DIR = os.path.join(os.path.expanduser("~"), "f1tenth_ws/src/f1tenth_gym_ros/maps")


def ensure_grayscale(map_name):
    """
    指定されたマップ画像を読み込み、白黒(Lモード)に変換して上書き保存する関数
    """
    try:
        # 拡張子なしの名前から、.png のパスを作成
        img_path = os.path.join(MAPS_DIR, map_name + ".png")

        if os.path.exists(img_path):
            img = Image.open(img_path)
            # 強制的に白黒(Lモード)に変換して上書き保存
            img.convert("L").save(img_path)
            print(f"[Auto-Convert] Converted {img_path} to Grayscale (L mode).")
        else:
            print(f"[Auto-Convert Warning] Image file not found: {img_path}")
    except Exception as e:
        print(f"[Auto-Convert Warning] Failed to convert map image: {e}")


def launch_setup(context, *args, **kwargs):
    """
    引数を受け取ってノードの設定を組み立てる関数
    """
    # 1. 起動引数からマップ名を取得（デフォルトは 'CourseImage_Standard'）
    map_name = LaunchConfiguration("map_name").perform(context)

    # 2. マップのパスを構築（拡張子なし）
    # 例: /home/samatsum/.../maps/CourseImage_Standard
    map_path = os.path.join(MAPS_DIR, map_name)

    # 3. 画像を白黒に変換（自動実行）
    ensure_grayscale(map_name)

    # 4. sim.yaml の読み込み（エージェント数などの設定確認用）
    f1tenth_gym_ros_share = get_package_share_directory("f1tenth_gym_ros")
    config_path = os.path.join(f1tenth_gym_ros_share, "config", "sim.yaml")
    config_dict = yaml.safe_load(open(config_path, "r"))

    has_opp = config_dict["bridge"]["ros__parameters"]["num_agent"] > 1

    # ---------------------------------------------------------
    # ノードの定義
    # ---------------------------------------------------------

    # Gym Bridge (シミュレータ本体)
    bridge_node = Node(
        package="f1tenth_gym_ros",
        executable="gym_bridge",
        name="bridge",
        parameters=[
            config_path,
            # ここでマップパスを上書き！
            {"map_path": map_path},
            {"map_img_ext": ".png"},
        ],
    )

    # Rviz2 (可視化)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=[
            "-d",
            os.path.join(f1tenth_gym_ros_share, "launch", "gym_bridge.rviz"),
        ],
    )

    # Map Server (地図配信)
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        parameters=[
            # ここでもマップパスを上書き！ (.yamlを付与)
            {"yaml_filename": map_path + ".yaml"},
            {"topic": "map"},
            {"frame_id": "map"},
            {"output": "screen"},
            {"use_sim_time": True},
        ],
    )

    # Lifecycle Manager (Map Serverの起動管理)
    nav_lifecycle_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    # Robot State Publisher (車のモデル表示 - 自車)
    ego_robot_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="ego_robot_state_publisher",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro ",
                        os.path.join(
                            f1tenth_gym_ros_share, "launch", "ego_racecar.xacro"
                        ),
                    ]
                )
            }
        ],
        remappings=[("/robot_description", "ego_robot_description")],
    )

    # Robot State Publisher (敵車 - 設定がある場合のみ)
    opp_robot_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="opp_robot_state_publisher",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro ",
                        os.path.join(
                            f1tenth_gym_ros_share, "launch", "opp_racecar.xacro"
                        ),
                    ]
                )
            }
        ],
        remappings=[("/robot_description", "opp_robot_description")],
    )

    # 実行するノードのリストを作成
    nodes_to_start = [
        bridge_node,
        rviz_node,
        map_server_node,
        nav_lifecycle_node,
        ego_robot_publisher,
    ]

    if has_opp:
        nodes_to_start.append(opp_robot_publisher)

    return nodes_to_start


def generate_launch_description():
    return LaunchDescription(
        [
            # 引数の定義（ここで map_name という引数を作っています）
            DeclareLaunchArgument(
                "map_name",
                default_value="CourseImage_Standard",  # 何も指定しない場合はこれが使われる
                description="Load map name without extension (e.g. my_course)",
            ),
            # 設定ロジックの呼び出し
            OpaqueFunction(function=launch_setup),
        ]
    )
