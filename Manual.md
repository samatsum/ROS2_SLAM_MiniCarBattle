F1TENTH シミュレーション ワークスペース 使い方まとめ
このワークスペースには2つのROS2パッケージがあります：

f1tenth_gym_ros - F1TENTHシミュレーション環境
wall_follow - 壁追従走行アルゴリズム
🔧 初回セットアップ / ビルド
ROS2環境の設定反映
bash
source /opt/ros/foxy/setup.bash
ワークスペースのビルド
bash
cd /home/samatsum/f1tenth_ws
colcon build
ビルド後の設定反映
bash
source /home/samatsum/f1tenth_ws/install/local_setup.bash
ヒント: 毎回これらを実行するのが面倒な場合は、~/.bashrc に追加しておくと便利です

🚗 シミュレータの起動
基本起動（デフォルトマップ: CourseImage_Standard）
bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
マップを指定して起動
bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py map_name:=マップ名
利用可能なマップ（拡張子なしで指定）:

マップ名	説明
CourseImage_Standard	デフォルトコース
Spielberg_map	シュピールベルクサーキット
levine	Levineマップ
例:

bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py map_name:=Spielberg_map
🤖 壁追従走行ノードの起動
シミュレータが起動している状態で、別のターミナルを開いて実行：

bash
source /opt/ros/foxy/setup.bash
source /home/samatsum/f1tenth_ws/install/local_setup.bash
ros2 launch wall_follow wall_follow.launch.py
スポーン位置を指定する場合
bash
ros2 launch wall_follow wall_follow.launch.py spawn_x:=5.0 spawn_y:=1.0 spawn_theta:=3.14
⌨️ キーボード操作（テレオペ）
config/sim.yaml
 の kb_teleop: True がデフォルトで有効になっています。

別のターミナルで実行：

bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
キー	動作
i	前進
u / o	前進+左右旋回
,	後退
m / .	後退+左右旋回
k	停止
⚙️ 設定ファイル
シミュレータ設定: 
config/sim.yaml
パラメータ	説明
num_agent	エージェント数（1 or 2）
sx, sy, stheta	自車の初期位置・向き
sx1, sy1, stheta1	対戦車の初期位置（2台モード時）
kb_teleop	キーボード操作の有効/無効
