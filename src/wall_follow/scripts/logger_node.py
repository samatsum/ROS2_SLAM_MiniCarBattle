#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from transforms3d.euler import quat2euler
import math
import numpy as np
import csv
import os
import datetime
import signal
import yaml
from PIL import Image, ImageDraw
from rcl_interfaces.srv import GetParameters


class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__("trajectory_logger")

        # --- 設定 ---
        self.wheelbase = 0.33  # 車体の長さ [m]
        self.log_dir = os.path.expanduser("~/f1tenth_ws/log/courseLog")
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

        # マップ設定 (YAMLファイルから自動読み込み)
        self.map_path = None
        self.map_settings = {
            "resolution": 0.01,
            "origin_x": 0.0,
            "origin_y": 0.0,
        }
        self.map_path_fetched = False
        self.param_client = None  # 遅延初期化

        # データバッファ
        self.history = []  # [timestamp, rx, ry, fx, fy, speed]
        self.scan_history = []  # [(timestamp, ranges, angle_min, angle_max, angle_increment)]

        # --- 通信 ---
        # 1. オドメトリ受信 (記録用)
        self.create_subscription(Odometry, "/ego_racecar/odom", self.odom_callback, 10)

        # 2. リセット信号受信 (保存トリガー) -> sim_reset_nodeが出すトピック
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.crash_callback, 10
        )

        # 3. Rviz用マーカー配信
        self.marker_pub = self.create_publisher(Marker, "/visual/trajectory", 10)
        
        # 4. LIDARスキャン受信 (線分抽出用)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)

        # 5. タイマー (0.1秒ごとに間引き記録)
        self.current_pose = None
        self.create_timer(0.1, self.timer_callback)
        
        # 6. マップパス取得タイマー (初期化をブロックしないよう遅延)
        self.create_timer(1.0, self.fetch_map_path_once)

        self.get_logger().info("Trajectory Logger Started. Press Ctrl+C to save and exit.")
    
    def fetch_map_path_once(self):
        """gym_bridgeノードからmap_pathパラメータを取得 (1回だけ実行)"""
        if self.map_path_fetched:
            return
        
        # サービスクライアントを遅延初期化
        if self.param_client is None:
            self.param_client = self.create_client(
                GetParameters, "/bridge/get_parameters"
            )
        
        # ノンブロッキングでサービスの確認
        if not self.param_client.service_is_ready():
            # サービスがまだ準備できていない場合は次のタイマーで再試行
            return
        
        request = GetParameters.Request()
        request.names = ["map_path"]
        
        future = self.param_client.call_async(request)
        future.add_done_callback(self.on_map_path_received)
        self.map_path_fetched = True
        self.get_logger().info("Fetching map_path from gym_bridge...")
    
    def on_map_path_received(self, future):
        """パラメータ取得完了時のコールバック"""
        try:
            result = future.result()
            if result.values:
                map_path_base = result.values[0].string_value
                # gym_bridge は拡張子なしで保存しているので .png を追加
                self.map_path = map_path_base + ".png"
                self.get_logger().info(f"Map path from gym_bridge: {self.map_path}")
                
                # マップのYAMLファイルから設定を読み込み
                yaml_path = map_path_base + ".yaml"
                self.load_map_settings(yaml_path)
            else:
                self.get_logger().warn("Could not get map_path from bridge node")
        except Exception as e:
            self.get_logger().error(f"Failed to get map_path: {e}")
    
    def load_map_settings(self, yaml_path):
        """マップのYAMLファイルから resolution と origin を読み込む"""
        try:
            if os.path.exists(yaml_path):
                with open(yaml_path, "r") as f:
                    map_yaml = yaml.safe_load(f)
                
                if "resolution" in map_yaml:
                    self.map_settings["resolution"] = float(map_yaml["resolution"])
                
                if "origin" in map_yaml:
                    origin = map_yaml["origin"]
                    if isinstance(origin, list) and len(origin) >= 2:
                        self.map_settings["origin_x"] = float(origin[0])
                        self.map_settings["origin_y"] = float(origin[1])
                
                self.get_logger().info(
                    f"Map settings loaded: resolution={self.map_settings['resolution']}, "
                    f"origin=({self.map_settings['origin_x']}, {self.map_settings['origin_y']})"
                )
            else:
                self.get_logger().warn(f"Map YAML not found: {yaml_path}, using defaults")
        except Exception as e:
            self.get_logger().error(f"Failed to load map settings: {e}")

    def odom_callback(self, msg):
        # 最初のオドメトリを受け取った瞬間に初期位置を記録
        if self.current_pose is None and len(self.history) == 0:
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            _, _, yaw = quat2euler([q.w, q.x, q.y, q.z])
            
            rx, ry = px, py
            fx = px + self.wheelbase * math.cos(yaw)
            fy = py + self.wheelbase * math.sin(yaw)
            
            timestamp = self.get_clock().now().nanoseconds / 1e9
            self.history.append([timestamp, rx, ry, fx, fy, 0.0])  # 速度0（静止）
            self.get_logger().info(f"Initial position recorded: ({rx:.3f}, {ry:.3f})")
        
        self.current_pose = msg

    def timer_callback(self):
        if self.current_pose is None:
            return

        px = self.current_pose.pose.pose.position.x
        py = self.current_pose.pose.pose.position.y

        q = self.current_pose.pose.pose.orientation
        _, _, yaw = quat2euler([q.w, q.x, q.y, q.z])

        rx, ry = px, py
        fx = px + self.wheelbase * math.cos(yaw)
        fy = py + self.wheelbase * math.sin(yaw)

        vx = self.current_pose.twist.twist.linear.x
        vy = self.current_pose.twist.twist.linear.y
        speed = math.sqrt(vx * vx + vy * vy)

        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.history.append([timestamp, rx, ry, fx, fy, speed])

        self.publish_markers()

    def publish_markers(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # 線の太さ
        marker.color.a = 1.0
        marker.color.r = 1.0  # 赤 (フロント)
        marker.color.g = 0.0
        marker.color.b = 0.0

        # フロントの軌跡だけ表示
        from geometry_msgs.msg import Point

        start_idx = max(0, len(self.history) - 200)
        for pt in self.history[start_idx:]:
            p = Point()
            p.x = pt[3]  # fx
            p.y = pt[4]  # fy
            marker.points.append(p)

        self.marker_pub.publish(marker)

    def scan_callback(self, msg):
        """LIDARスキャンコールバック (10Hzで間引き保存)"""
        # 10回に1回だけ保存 (約1Hz)
        if not hasattr(self, '_scan_count'):
            self._scan_count = 0
        self._scan_count += 1
        if self._scan_count % 10 != 0:
            return
        
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.scan_history.append((
            timestamp,
            list(msg.ranges),
            msg.angle_min,
            msg.angle_max,
            msg.angle_increment
        ))

    def crash_callback(self, msg):
        """クラッシュ時のコールバック"""
        self.save_log("crash")

    def save_log(self, reason="manual"):
        """ログを保存する共通関数
        
        Args:
            reason: ログ出力理由 ("crash", "manual", etc.)
        """
        if not self.history:
            self.get_logger().warn("No data to save.")
            return

        self.get_logger().warn(f"Saving log data... (reason: {reason})")
        now_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # ディレクトリ名: 日付時間_理由
        run_dir_name = f"{now_str}_{reason}"
        run_dir = os.path.join(self.log_dir, run_dir_name)
        
        # サブディレクトリ作成 (simulation は不要なので削除)
        images_dir = os.path.join(run_dir, "images")
        mickey_dir = os.path.join(run_dir, "mickey")
        os.makedirs(images_dir, exist_ok=True)
        os.makedirs(mickey_dir, exist_ok=True)

        # 2. mickey/ミッキーデータ生成 (500Hz想定、ローカル座標系)
        front_mickey = []
        rear_mickey = []
        mickey_per_meter = 1000
        
        # --- 初期位置の保存 ---
        if len(self.history) > 0:
            init_pose_path = os.path.join(run_dir, "initial_pose.yaml")
            try:
                # self.history[0] = [timestamp, rx, ry, fx, fy, speed]
                # yawは後輪→前輪ベクトルから計算
                rx0, ry0 = self.history[0][1], self.history[0][2]
                fx0, fy0 = self.history[0][3], self.history[0][4]
                initial_yaw = math.atan2(fy0 - ry0, fx0 - rx0)
                
                init_data = {
                    "timestamp": float(self.history[0][0]),
                    "rx": float(rx0),
                    "ry": float(ry0),
                    "fx": float(fx0),
                    "fy": float(fy0),
                    "yaw_rad": float(initial_yaw),
                    "yaw_deg": float(math.degrees(initial_yaw)),
                }
                
                with open(init_pose_path, "w") as f:
                    yaml.dump(init_data, f)
                self.get_logger().info(f"Saved initial pose: {init_pose_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to save initial pose: {e}")
        # ----------------------
        try:
            front_mickey, rear_mickey = self.generate_mickey_data(mickey_dir)
        except Exception as e:
            self.get_logger().error(f"Failed to generate mickey data: {e}")

        # DR経路計算（ミッキーデータから）
        dr_path = []
        if front_mickey and rear_mickey:
            dr_path = self.calculate_dead_reckoning_path(front_mickey, rear_mickey, mickey_per_meter)

        # 3. images/trajectory_map.png 画像生成
        try:
            map_path = self.map_path
            if not map_path:
                self.get_logger().warn("Map path not yet fetched from gym_bridge. Skipping image generation.")
            elif os.path.exists(map_path):
                res = self.map_settings["resolution"]
                org_x = self.map_settings["origin_x"]
                org_y = self.map_settings["origin_y"]

                img = Image.open(map_path).convert("RGB")
                draw = ImageDraw.Draw(img)
                width, height = img.size

                def to_pix(mx, my):
                    px = (mx - org_x) / res
                    py = height - ((my - org_y) / res)
                    return px, py

                # 軌跡の描画
                for i in range(len(self.history) - 1):
                    r1 = to_pix(self.history[i][1], self.history[i][2])
                    r2 = to_pix(self.history[i + 1][1], self.history[i + 1][2])
                    draw.line([r1, r2], fill=(0, 0, 255), width=2)
                    f1 = to_pix(self.history[i][3], self.history[i][4])
                    f2 = to_pix(self.history[i + 1][3], self.history[i + 1][4])
                    draw.line([f1, f2], fill=(255, 0, 0), width=2)

                # 開始点を緑で描画
                px_pos, py_pos = 0, 0
                if len(self.history) > 0:
                    pt = self.history[0]
                    center_x = (pt[1] + pt[3]) / 2.0
                    center_y = (pt[2] + pt[4]) / 2.0
                    px_pos, py_pos = to_pix(center_x, center_y)
                    draw.ellipse(
                        [px_pos - 15, py_pos - 15, px_pos + 15, py_pos + 15],
                        fill=(0, 255, 0),
                        outline=(0, 200, 0)
                    )

                png_path = os.path.join(images_dir, "trajectory_map.png")
                img.save(png_path)
                self.get_logger().info(f"Saved: {png_path}")
                
                # 4. images/kinematics.png 生成（DR経路の可視化）
                if dr_path:
                    img_kin = Image.open(map_path).convert("RGB")
                    draw_kin = ImageDraw.Draw(img_kin)
                    
                    # オドメトリ軌跡を描画（薄い色）
                    for i in range(len(self.history) - 1):
                        r1 = to_pix(self.history[i][1], self.history[i][2])
                        r2 = to_pix(self.history[i + 1][1], self.history[i + 1][2])
                        draw_kin.line([r1, r2], fill=(150, 150, 255), width=1)
                        f1 = to_pix(self.history[i][3], self.history[i][4])
                        f2 = to_pix(self.history[i + 1][3], self.history[i + 1][4])
                        draw_kin.line([f1, f2], fill=(255, 150, 150), width=1)
                    
                    # DRパスをワールド座標に変換するための初期位置・初期向き
                    if len(self.history) > 0:
                        init_rx, init_ry = self.history[0][1], self.history[0][2]
                        init_fx, init_fy = self.history[0][3], self.history[0][4]
                        init_theta = math.atan2(init_fy - init_ry, init_fx - init_rx)
                        cos_init = math.cos(init_theta)
                        sin_init = math.sin(init_theta)
                    else:
                        init_rx, init_ry = 0.0, 0.0
                        cos_init, sin_init = 1.0, 0.0
                    
                    def dr_to_world(dr_x, dr_y):
                        """正規化座標 → ワールド座標"""
                        world_x = dr_x * cos_init - dr_y * sin_init + init_rx
                        world_y = dr_x * sin_init + dr_y * cos_init + init_ry
                        return world_x, world_y
                    
                    # DR経路を描画（シアン色）- ワールド座標に変換
                    step = max(1, len(dr_path) // 5000)
                    for i in range(0, len(dr_path) - step, step):
                        p1 = dr_path[i]
                        p2 = dr_path[min(i + step, len(dr_path) - 1)]
                        w1_x, w1_y = dr_to_world(p1[1], p1[2])
                        w2_x, w2_y = dr_to_world(p2[1], p2[2])
                        px1, py1 = to_pix(w1_x, w1_y)
                        px2, py2 = to_pix(w2_x, w2_y)
                        draw_kin.line([(px1, py1), (px2, py2)], fill=(0, 255, 255), width=3)
                    
                    # 開始点（緑）と終了点（マゼンタ）
                    if len(self.history) > 0:
                        draw_kin.ellipse([px_pos - 12, py_pos - 12, px_pos + 12, py_pos + 12], fill=(0, 255, 0), outline=(0, 180, 0))
                    if dr_path:
                        end_w_x, end_w_y = dr_to_world(dr_path[-1][1], dr_path[-1][2])
                        end_px, end_py = to_pix(end_w_x, end_w_y)
                        draw_kin.ellipse([end_px - 10, end_py - 10, end_px + 10, end_py + 10], fill=(255, 0, 255), outline=(180, 0, 180))
                    
                    kin_path = os.path.join(images_dir, "kinematics.png")
                    img_kin.save(kin_path)
                    self.get_logger().info(f"Saved: {kin_path}")
            else:
                self.get_logger().error(f"Map file not found: {map_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to generate image: {e}")

        # 5. images/mickey_lidar_points.png 生成（DR経路を使用）
        try:
            if dr_path:
                self.generate_lidar_points_from_dr(images_dir, dr_path)
            else:
                self.get_logger().warn("Skipping LIDAR points generation due to missing DR path")
        except Exception as e:
            self.get_logger().error(f"Failed to generate LIDAR points: {e}")

        # 履歴クリア
        self.history = []
        self.scan_history = []
    
    def calculate_dead_reckoning_path(self, front_mickey, rear_mickey, mickey_per_meter):
        """Dead Reckoning：正規化済みミッキーデータからDRパスを生成
        
        ミッキーデータは既に正規化座標系（初期向き=0度）で記録されている。
        
        【重要】self.historyへの依存なし - 本番環境で動作可能
        
        座標系の定義：
        - 原点 = 初期後輪位置 (0, 0)
        - 初期方向 = 0度（後輪→前輪ベクトルの方向 = +X方向）
        
        改善点：
        - 浮動小数点Total値（列5,6）を使用して丸め誤差を排除
        - 角速度積分方式でtheta計算
        - ループ最適化：逆数の事前計算、前回値の保持
        """
        if len(front_mickey) != len(rear_mickey) or len(rear_mickey) < 2:
            return []

        dr_path = []
        theta = 0.0  # 初期向きは0度
        
        # 最適化：逆数を事前計算（割り算→掛け算）
        inv_mpm = 1.0 / mickey_per_meter
        inv_wb = 1.0 / self.wheelbase
        
        # 初期値を計算
        prev_rear_x = rear_mickey[0][5] * inv_mpm
        prev_rear_y = rear_mickey[0][6] * inv_mpm
        prev_front_x = front_mickey[0][5] * inv_mpm + self.wheelbase
        prev_front_y = front_mickey[0][6] * inv_mpm

        for i in range(len(rear_mickey)):
            timestamp = rear_mickey[i][0]
            
            # 浮動小数点Total値を使用（掛け算で高速化）
            rear_x = rear_mickey[i][5] * inv_mpm
            rear_y = rear_mickey[i][6] * inv_mpm
            front_x = front_mickey[i][5] * inv_mpm + self.wheelbase
            front_y = front_mickey[i][6] * inv_mpm
            
            if i > 0:
                # 前回との差分（変位）を計算
                rear_dx = rear_x - prev_rear_x
                rear_dy = rear_y - prev_rear_y
                front_dx = front_x - prev_front_x
                front_dy = front_y - prev_front_y
                
                # 角速度積分方式でtheta計算
                cos_t = math.cos(theta)
                sin_t = math.sin(theta)
                
                # 横方向変位を計算
                rear_lateral = -rear_dx * sin_t + rear_dy * cos_t
                front_lateral = -front_dx * sin_t + front_dy * cos_t
                
                # 角速度 = (前輪横変位 - 後輪横変位) / ホイールベース
                dtheta = (front_lateral - rear_lateral) * inv_wb
                theta += dtheta
            
            # 前回値を更新（次のループで使用）
            prev_rear_x, prev_rear_y = rear_x, rear_y
            prev_front_x, prev_front_y = front_x, front_y
            
            # 後輪位置を出力
            dr_path.append([timestamp, rear_x, rear_y, theta])

        return dr_path
    
    def generate_lidar_points_from_dr(self, images_dir, dr_path):
        """DR経路を使ってLIDAR点群を可視化（0.07m以内の点は線でつなぐ）
        
        【重要】self.historyへの依存なし - 本番環境で動作可能
        """
        if len(self.scan_history) < 2 or not dr_path:
            self.get_logger().warn("Not enough data for LIDAR points")
            return
        
        all_segments = []
        # 開始時刻はスキャン履歴の最初のタイムスタンプから取得（self.history不要）
        start_time = self.scan_history[0][0] if self.scan_history else 0
        
        # 線分接続の閾値（ユーザー指定: 4m * sin(1deg) approx 0.07m）
        connect_threshold = 0.07
        
        for scan_entry in self.scan_history:
            scan_time, ranges, angle_min, angle_max, angle_inc = scan_entry
            
            # 最も近い時刻のポーズを見つける
            closest_pose = None
            min_diff = float('inf')
            # 単純な線形探索だと遅いので、前回のインデックスから探すなどの工夫が可能だが
            # ここではデータ量がそれほどでもないのでループで処理
            for pose in dr_path:
                # pose[0] is timestamp relative to start? or absolute?
                # dr_path generation code uses `timestamp` directly from mickey data
                # mickey data timestamp is `t` (relative?) -> No, it's relative in generate_mickey_data `while t <= end_time`.
                # Wait, scan_time is absolute (nanoseconds/1e9).
                # dr_path timestamp needs to be aligned. 
                # In calculate_dead_reckoning_path, timestamp comes from rear_mickey[i][0] which is RELATIVE `round(t, 4)`.
                # So we need to compare `scan_time - start_time` with `pose[0]`.
                
                diff = abs(pose[0] - (scan_time - start_time))
                if diff < min_diff:
                    min_diff = diff
                    closest_pose = pose
            
            if closest_pose is None:
                continue
            
            _, x, y, theta = closest_pose
            
            # DRパスは後輪位置を出力しているので、LIDARオフセットを適用
            lidar_offset = 0.275
            sensor_x = x + lidar_offset * math.cos(theta)
            sensor_y = y + lidar_offset * math.sin(theta)
            
            # NumPyベクトル化：このスキャンの全点を一括でワールド座標に変換
            ranges_arr = np.array(ranges)
            n_points = len(ranges)
            
            # 角度配列を生成
            angles = angle_min + np.arange(n_points) * angle_inc
            
            # 有効な点のマスク
            valid_mask = (ranges_arr >= 0.1) & (ranges_arr <= 10.0) & np.isfinite(ranges_arr)
            
            # ワールド座標を一括計算
            world_angles = theta + angles
            wx_all = sensor_x + ranges_arr * np.cos(world_angles)
            wy_all = sensor_y + ranges_arr * np.sin(world_angles)
            
            # 隣接点をつなぐセグメントを生成（ベクトル化）
            # 両方の点が有効で、距離が閾値以下の場合のみ接続
            for i in range(n_points - 1):
                if valid_mask[i] and valid_mask[i + 1]:
                    dx = wx_all[i + 1] - wx_all[i]
                    dy = wy_all[i + 1] - wy_all[i]
                    dist = math.sqrt(dx * dx + dy * dy)
                    if dist <= connect_threshold:
                        all_segments.append([(wx_all[i], wy_all[i]), (wx_all[i + 1], wy_all[i + 1])])

        if not all_segments:
            self.get_logger().warn("No valid LIDAR segments generated")
            return
        
        # プロット作成
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection
        
        fig, ax = plt.subplots(figsize=(12, 8))
        
        # LineCollectionで高速描画
        lc = LineCollection(all_segments, colors='blue', linewidths=0.5, alpha=0.5)
        ax.add_collection(lc)
        
        # スケール調整用に全点を集める
        all_x = []
        all_y = []
        # 全点は多すぎるので、セグメントの間引きサンプリングで範囲を決める
        step = max(1, len(all_segments) // 1000)
        for i in range(0, len(all_segments), step):
            seg = all_segments[i]
            all_x.append(seg[0][0])
            all_x.append(seg[1][0])
            all_y.append(seg[0][1])
            all_y.append(seg[1][1])
            
        if dr_path:
            ax.scatter([dr_path[0][1]], [dr_path[0][2]], c='green', s=100, marker='o', label='Start')
        
        if all_x:
            margin = 2.0
            ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
            ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
            
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_title(f'LIDAR Wall Map (Segs < {connect_threshold}m)')
        ax.set_aspect('equal')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        png_path = os.path.join(images_dir, "mickey_lidar_points.png")
        plt.savefig(png_path, dpi=150, bbox_inches='tight')
        plt.close()
        self.get_logger().info(f"Saved: {png_path}")

    def generate_mickey_data(self, run_dir):
        """車体の前輪・後輪を2つのマウスとして扱い、ミッキーデータを生成する
        
        500Hz (2msごと) でサンプリングしたデータを生成。
        
        正規化座標系での変位を記録（初期位置=原点、初期向き=0度）：
        - ワールド座標変位を初期向きで回転して正規化
        - Total_X, Total_Yは正規化座標系での累積値
        
        【シミュレーション用】self.historyから初期向きを取得して正規化
        【本番環境】マウスセンサーが直接ローカル座標を出力するため、この変換は不要
        """
        if len(self.history) < 2:
            self.get_logger().warn("Not enough data to generate mickey data")
            return [], []
        
        start_time = self.history[0][0]
        
        # 初期向きを取得して正規化用の回転行列を準備
        rx0, ry0 = self.history[0][1], self.history[0][2]
        fx0, fy0 = self.history[0][3], self.history[0][4]
        initial_theta = math.atan2(fy0 - ry0, fx0 - rx0)
        cos_init = math.cos(-initial_theta)
        sin_init = math.sin(-initial_theta)
        
        def interpolate_position(t, history, idx_x, idx_y):
            for i in range(len(history) - 1):
                t0 = history[i][0] - start_time
                t1 = history[i + 1][0] - start_time
                if t0 <= t <= t1:
                    if t1 == t0:
                        return history[i][idx_x], history[i][idx_y]
                    ratio = (t - t0) / (t1 - t0)
                    x = history[i][idx_x] + ratio * (history[i + 1][idx_x] - history[i][idx_x])
                    y = history[i][idx_y] + ratio * (history[i + 1][idx_y] - history[i][idx_y])
                    return x, y
            return history[-1][idx_x], history[-1][idx_y]
        
        end_time = self.history[-1][0] - start_time
        dt = 0.002  # 500Hz
        mickey_per_meter = 1000
        
        # 前輪ミッキーデータ（正規化座標系）
        front_mickey = []
        prev_fx, prev_fy = interpolate_position(0, self.history, 3, 4)
        total_fx_float, total_fy_float = 0.0, 0.0
        
        t = 0.0
        while t <= end_time:
            fx, fy = interpolate_position(t, self.history, 3, 4)
            
            # ワールド座標変位を正規化座標に変換
            dx_world = fx - prev_fx
            dy_world = fy - prev_fy
            dx_norm = dx_world * cos_init - dy_world * sin_init
            dy_norm = dx_world * sin_init + dy_world * cos_init
            
            rel_x_float = dx_norm * mickey_per_meter
            rel_y_float = dy_norm * mickey_per_meter
            total_fx_float += rel_x_float
            total_fy_float += rel_y_float
            
            rel_x = int(round(rel_x_float))
            rel_y = int(round(rel_y_float))
            total_fx = int(round(total_fx_float))
            total_fy = int(round(total_fy_float))
            
            # 浮動小数点Total値を追加（精度向上のため）
            front_mickey.append([round(t, 4), rel_x, rel_y, total_fx, total_fy, 
                                round(total_fx_float, 4), round(total_fy_float, 4)])
            prev_fx, prev_fy = fx, fy
            t += dt
        
        # 後輪ミッキーデータ（正規化座標系）
        rear_mickey = []
        prev_rx, prev_ry = interpolate_position(0, self.history, 1, 2)
        total_rx_float, total_ry_float = 0.0, 0.0
        
        t = 0.0
        while t <= end_time:
            rx, ry = interpolate_position(t, self.history, 1, 2)
            
            # ワールド座標変位を正規化座標に変換
            dx_world = rx - prev_rx
            dy_world = ry - prev_ry
            dx_norm = dx_world * cos_init - dy_world * sin_init
            dy_norm = dx_world * sin_init + dy_world * cos_init
            
            rel_x_float = dx_norm * mickey_per_meter
            rel_y_float = dy_norm * mickey_per_meter
            total_rx_float += rel_x_float
            total_ry_float += rel_y_float
            
            rel_x = int(round(rel_x_float))
            rel_y = int(round(rel_y_float))
            total_rx = int(round(total_rx_float))
            total_ry = int(round(total_ry_float))
            
            # 浮動小数点Total値を追加（精度向上のため）
            rear_mickey.append([round(t, 4), rel_x, rel_y, total_rx, total_ry,
                               round(total_rx_float, 4), round(total_ry_float, 4)])
            prev_rx, prev_ry = rx, ry
            t += dt
        
        # CSVに保存（浮動小数点Total値を含む）
        front_mickey_path = os.path.join(run_dir, "mickey_front.csv")
        with open(front_mickey_path, "w") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp_s", "Rel_X", "Rel_Y", "Total_X", "Total_Y", "Total_X_f", "Total_Y_f"])
            writer.writerows(front_mickey)
        self.get_logger().info(f"Saved Front Mickey Data: {front_mickey_path}")
        
        rear_mickey_path = os.path.join(run_dir, "mickey_rear.csv")
        with open(rear_mickey_path, "w") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp_s", "Rel_X", "Rel_Y", "Total_X", "Total_Y", "Total_X_f", "Total_Y_f"])
            writer.writerows(rear_mickey)
        self.get_logger().info(f"Saved Rear Mickey Data: {rear_mickey_path}")
        
        self.calculate_vehicle_kinematics(run_dir, front_mickey, rear_mickey, mickey_per_meter)
        
        return front_mickey, rear_mickey

    def calculate_vehicle_kinematics(self, run_dir, front_mickey, rear_mickey, mickey_per_meter):
        """前輪・後輪のミッキーデータから車体の移動距離と回転角度を計算
        
        ユーザーのアイデアに基づく計算:
        - 後輪と前輪の累積位置（Total_X, Total_Y）から車体の向きθを計算
        - θ = atan2(front_Y - rear_Y, front_X - rear_X)
        - Rotation_deg = 現在のθ - 初期のθ
        """
        if len(front_mickey) != len(rear_mickey):
            self.get_logger().error("Front and rear mickey data length mismatch")
            return
        
        # 初期の車体向き（オドメトリから取得）
        if len(self.history) > 0:
            rx0, ry0 = self.history[0][1], self.history[0][2]
            fx0, fy0 = self.history[0][3], self.history[0][4]
            initial_theta = math.atan2(fy0 - ry0, fx0 - rx0)
        else:
            rx0, ry0 = 0.0, 0.0
            fx0, fy0 = self.wheelbase, 0.0
            initial_theta = 0.0
        
        kinematics_data = []
        total_distance = 0.0
        
        for i in range(len(front_mickey)):
            timestamp = front_mickey[i][0]
            
            # 前輪・後輪の累積位置（初期位置 + ミッキー累積変位）
            front_total_x = fx0 + front_mickey[i][3] / mickey_per_meter
            front_total_y = fy0 + front_mickey[i][4] / mickey_per_meter
            rear_total_x = rx0 + rear_mickey[i][3] / mickey_per_meter
            rear_total_y = ry0 + rear_mickey[i][4] / mickey_per_meter
            
            # 車体の向き θ = atan2(前輪位置 - 後輪位置)
            dx = front_total_x - rear_total_x
            dy = front_total_y - rear_total_y
            
            if abs(dx) > 0.001 or abs(dy) > 0.001:
                current_theta = math.atan2(dy, dx)
            else:
                current_theta = initial_theta
            
            # 車体の移動距離（後輪の移動量）
            if i > 0:
                prev_rear_x = rx0 + rear_mickey[i-1][3] / mickey_per_meter
                prev_rear_y = ry0 + rear_mickey[i-1][4] / mickey_per_meter
                # current rear pos
                curr_rx = rx0 + rear_mickey[i][3] / mickey_per_meter
                curr_ry = ry0 + rear_mickey[i][4] / mickey_per_meter
                
                distance_step = math.sqrt((curr_rx - prev_rear_x)**2 + (curr_ry - prev_rear_y)**2)
            else:
                distance_step = 0.0
            
            total_distance += distance_step
            
            # Rotation: 初期向きからの差分を計算（累積回転）
            # atan2の不連続性を考慮する必要があるが、単純差分で可視化するには十分か？
            # ここではシンプルに角度差を出す
            theta_diff = current_theta - initial_theta
            # Normalize to -pi to pi? Or keep accumulated?
            # ユーザーの要望は「車体の回転」。累積回転の方が分かりやすい場合も。
            # しかしatan2は -pi~pi なので、unwrapしないとジャンプする。
            # 今回はシンプルに current_theta - initial_theta を度数法で出力
            
            # Wrap to -180 to 180 for standard heading deviation
            while theta_diff > math.pi: theta_diff -= 2*math.pi
            while theta_diff <= -math.pi: theta_diff += 2*math.pi
            
            rotation_deg = math.degrees(theta_diff)
            
            kinematics_data.append([
                timestamp,
                round(distance_step * 1000, 3),   # Distance_mm
                round(total_distance * 1000, 1),  # Total_Distance_mm
                round(rotation_deg, 2)            # Rotation_deg
            ])
        
        # CSVに保存
        kinematics_path = os.path.join(run_dir, "mickey_kinematics.csv")
        with open(kinematics_path, "w") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp_s", "Distance_mm", "Total_Distance_mm", "Rotation_deg"])
            writer.writerows(kinematics_data)
        self.get_logger().info(f"Saved Vehicle Kinematics: {kinematics_path}")

    def generate_lidar_segments_mickey(self, mickey_dir, images_dir):
        """ミッキーデータに基づいてLIDARスキャンから線分を抽出
        
        mickey_kinematics.csvの移動距離とステアリング角度を使って
        Dead Reckoningで位置を推定し、各スキャンを変換して線分を抽出
        """
        if len(self.scan_history) < 2:
            self.get_logger().warn("Not enough scan data for mickey-based segments")
            return
        
        if len(self.history) < 2:
            self.get_logger().warn("Not enough odometry data for mickey-based segments")
            return
        
        # Dead Reckoningで位置を推定
        # history: [timestamp, rx, ry, fx, fy, speed]
        # スキャンのタイムスタンプに対応する位置を使用
        
        all_segments = []
        distance_threshold = 0.07  # 線分抽出の閾値
        
        for scan_data in self.scan_history:
            scan_time, ranges, angle_min, angle_max, angle_inc = scan_data
            
            # スキャン時刻に最も近いオドメトリを探す
            closest_odom = None
            min_time_diff = float('inf')
            for odom_point in self.history:
                time_diff = abs(odom_point[0] - scan_time)
                if time_diff < min_time_diff:
                    min_time_diff = time_diff
                    closest_odom = odom_point
            
            if closest_odom is None or min_time_diff > 1.0:
                continue
            
            # ロボット位置（後輪基準）
            rx, ry = closest_odom[1], closest_odom[2]
            fx, fy = closest_odom[3], closest_odom[4]
            theta = math.atan2(fy - ry, fx - rx)
            
            # スキャンをポイントクラウドに変換
            ranges_arr = np.array(ranges)
            num_points = len(ranges_arr)
            angles = np.linspace(angle_min, angle_max, num_points)
            
            valid = np.isfinite(ranges_arr) & (ranges_arr > 0.01) & (ranges_arr < 10.0)
            r = ranges_arr[valid]
            a = angles[valid]
            
            if len(r) < 3:
                continue
            
            # ロボット座標系でのポイント
            local_x = r * np.cos(a)
            local_y = r * np.sin(a)
            
            # ワールド座標系に変換
            c = math.cos(theta)
            s = math.sin(theta)
            world_x = c * local_x - s * local_y + rx
            world_y = s * local_x + c * local_y + ry
            
            points = np.column_stack([world_x, world_y])
            
            # 線分抽出 (0.07m閾値)
            current_group = [points[0]]
            for i in range(1, len(points)):
                dist = np.linalg.norm(points[i] - points[i-1])
                if dist <= distance_threshold:
                    current_group.append(points[i])
                else:
                    if len(current_group) >= 3:
                        start = current_group[0]
                        end = current_group[-1]
                        length = np.linalg.norm(end - start)
                        if length >= 0.1:
                            angle = math.atan2(end[1] - start[1], end[0] - start[0])
                            all_segments.append([start[0], start[1], end[0], end[1], length, angle])
                    current_group = [points[i]]
            
            # 最後のグループ
            if len(current_group) >= 3:
                start = current_group[0]
                end = current_group[-1]
                length = np.linalg.norm(end - start)
                if length >= 0.1:
                    angle = math.atan2(end[1] - start[1], end[0] - start[0])
                    all_segments.append([start[0], start[1], end[0], end[1], length, angle])
        
        # CSVに保存
        segments_path = os.path.join(mickey_dir, "mickey_lidar_segments.csv")
        with open(segments_path, "w") as f:
            writer = csv.writer(f)
            writer.writerow(["start_x", "start_y", "end_x", "end_y", "length", "angle"])
            writer.writerows(all_segments)
        self.get_logger().info(f"Saved Mickey-based Segments: {segments_path} ({len(all_segments)} segments)")
        
        # PNG可視化を生成
        try:
            import matplotlib
            matplotlib.use('Agg')  # GUIなしで動作させる
            import matplotlib.pyplot as plt
            from matplotlib.collections import LineCollection
            
            fig, ax = plt.subplots(figsize=(12, 12))
            
            # 線分データをプロット用に変換
            if all_segments:
                lines = [[(seg[0], seg[1]), (seg[2], seg[3])] for seg in all_segments]
                lc = LineCollection(lines, colors='blue', linewidths=1.5, alpha=0.8)
                ax.add_collection(lc)
                
                # 軸範囲を計算
                all_x = [seg[0] for seg in all_segments] + [seg[2] for seg in all_segments]
                all_y = [seg[1] for seg in all_segments] + [seg[3] for seg in all_segments]
                margin = 2.0
                ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
                ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
            
            ax.set_aspect('equal')
            ax.grid(True, linestyle=':', alpha=0.6)
            ax.set_title(f"Mickey LIDAR Segments Map\n{len(all_segments)} segments")
            ax.set_xlabel("X [m]")
            ax.set_ylabel("Y [m]")
            
            # 車体のスタート位置をマーク (self.history[0] = 最初のオドメトリ位置)
            if len(self.history) > 0:
                start_x = self.history[0][1]  # rear_x
                start_y = self.history[0][2]  # rear_y
                ax.plot(start_x, start_y, 'go', markersize=10, label='Start Position')
            ax.legend()
            
            # 保存
            png_path = os.path.join(images_dir, "mickey_lidar_segments.png")
            plt.savefig(png_path, dpi=150, bbox_inches='tight')
            plt.close()
            self.get_logger().info(f"Saved Segments Visualization: {png_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to generate segments PNG: {e}")


# グローバル変数（シグナルハンドラ用）
_node = None


def signal_handler(sig, frame):
    """Ctrl+C が押された時のハンドラ"""
    global _node
    if _node is not None:
        _node.get_logger().warn("Ctrl+C detected! Saving log before exit...")
        _node.save_log("manual")
    rclpy.shutdown()


def main(args=None):
    global _node
    rclpy.init(args=args)
    _node = TrajectoryLogger()
    
    # Ctrl+C のシグナルハンドラを登録
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        rclpy.spin(_node)
    except Exception:
        pass
    finally:
        if _node is not None:
            _node.destroy_node()


if __name__ == "__main__":
    main()
