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
        self.current_pose = msg

    def timer_callback(self):
        if self.current_pose is None:
            return

        # 座標計算
        px = self.current_pose.pose.pose.position.x
        py = self.current_pose.pose.pose.position.y

        # クォータニオン -> ヨー角
        q = self.current_pose.pose.pose.orientation
        _, _, yaw = quat2euler([q.w, q.x, q.y, q.z])

        # リア(後輪) = base_link位置 (f1tenthの仕様)
        rx, ry = px, py

        # フロント(前輪) = base_link + wheelbase * 回転
        fx = px + self.wheelbase * math.cos(yaw)
        fy = py + self.wheelbase * math.sin(yaw)

        # 速度計算 (linear velocity の絶対値)
        vx = self.current_pose.twist.twist.linear.x
        vy = self.current_pose.twist.twist.linear.y
        speed = math.sqrt(vx * vx + vy * vy)

        # 記録
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.history.append([timestamp, rx, ry, fx, fy, speed])

        # Rviz可視化 (最新の100点だけ表示して軽量化)
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
        
        # サブディレクトリ作成
        images_dir = os.path.join(run_dir, "images")
        simulation_dir = os.path.join(run_dir, "simulation")
        mickey_dir = os.path.join(run_dir, "mickey")
        os.makedirs(images_dir, exist_ok=True)
        os.makedirs(simulation_dir, exist_ok=True)
        os.makedirs(mickey_dir, exist_ok=True)

        # 1. simulation/simulation_position.csv 保存
        csv_path = os.path.join(simulation_dir, "simulation_position.csv")
        with open(csv_path, "w") as f:
            writer = csv.writer(f)
            writer.writerow(["time", "rear_x", "rear_y", "front_x", "front_y", "speed"])
            writer.writerows(self.history)
        self.get_logger().info(f"Saved: {csv_path}")

        # 2. mickey/ミッキーデータ生成 (500Hz想定)
        try:
            self.generate_mickey_data(mickey_dir)
        except Exception as e:
            self.get_logger().error(f"Failed to generate mickey data: {e}")

        # 3. images/trajectory_map.png 画像生成
        try:
            map_path = self.map_path
            if not map_path:
                self.get_logger().warn("Map path not yet fetched from gym_bridge. Skipping image generation.")
            elif os.path.exists(map_path):
                # YAMLファイルから読み込んだ設定を使用
                res = self.map_settings["resolution"]
                org_x = self.map_settings["origin_x"]
                org_y = self.map_settings["origin_y"]

                img = Image.open(map_path).convert("RGB")
                draw = ImageDraw.Draw(img)
                width, height = img.size

                # 座標変換関数 (Map -> Pixel)
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
            else:
                self.get_logger().error(f"Map file not found: {map_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to generate image: {e}")

        # 4. mickey/mickey_lidar_segments.csv 生成
        try:
            self.generate_lidar_segments_mickey(mickey_dir)
        except Exception as e:
            self.get_logger().error(f"Failed to generate mickey-based segments: {e}")

        # 履歴クリア
        self.history = []
        self.scan_history = []

    def generate_mickey_data(self, run_dir):
        """車体の前輪・後輪を2つのマウスとして扱い、ミッキーデータを生成する
        
        500Hz (2msごと) でサンプリングしたデータを生成。
        """
        if len(self.history) < 2:
            self.get_logger().warn("Not enough data to generate mickey data")
            return
        
        # 開始時刻を0にオフセット
        start_time = self.history[0][0]
        
        # 線形補間で500Hzにリサンプリング
        def interpolate_position(t, history, idx_x, idx_y):
            """時刻tにおける位置を線形補間で取得"""
            # 該当区間を探す
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
            # 範囲外の場合は最後の値
            return history[-1][idx_x], history[-1][idx_y]
        
        # 総時間を計算
        end_time = self.history[-1][0] - start_time
        dt = 0.002  # 500Hz = 2ms
        
        # ミッキーデータ生成（DPIを仮に1000と仮定: 1m = 1000 mickey / 0.0254 = 約39370 mickey）
        # 簡略化: 1mm = 1 mickey (DPI依存だが、理解しやすい値として設定)
        mickey_per_meter = 1000  # 1m = 1000 mickey (1mm = 1 mickey)
        
        # 前輪ミッキーデータ
        front_mickey = []
        prev_fx, prev_fy = interpolate_position(0, self.history, 3, 4)
        total_fx, total_fy = 0, 0
        
        t = 0.0
        while t <= end_time:
            fx, fy = interpolate_position(t, self.history, 3, 4)
            rel_x = int((fx - prev_fx) * mickey_per_meter)
            rel_y = int((fy - prev_fy) * mickey_per_meter)
            total_fx += rel_x
            total_fy += rel_y
            front_mickey.append([round(t, 4), rel_x, rel_y, total_fx, total_fy])
            prev_fx, prev_fy = fx, fy
            t += dt
        
        # 後輪ミッキーデータ
        rear_mickey = []
        prev_rx, prev_ry = interpolate_position(0, self.history, 1, 2)
        total_rx, total_ry = 0, 0
        
        t = 0.0
        while t <= end_time:
            rx, ry = interpolate_position(t, self.history, 1, 2)
            rel_x = int((rx - prev_rx) * mickey_per_meter)
            rel_y = int((ry - prev_ry) * mickey_per_meter)
            total_rx += rel_x
            total_ry += rel_y
            rear_mickey.append([round(t, 4), rel_x, rel_y, total_rx, total_ry])
            prev_rx, prev_ry = rx, ry
            t += dt
        
        # 前輪ミッキーデータをCSVに保存
        front_mickey_path = os.path.join(run_dir, "mickey_front.csv")
        with open(front_mickey_path, "w") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp_s", "Rel_X", "Rel_Y", "Total_X", "Total_Y"])
            writer.writerows(front_mickey)
        self.get_logger().info(f"Saved Front Mickey Data: {front_mickey_path}")
        
        # 後輪ミッキーデータをCSVに保存
        rear_mickey_path = os.path.join(run_dir, "mickey_rear.csv")
        with open(rear_mickey_path, "w") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp_s", "Rel_X", "Rel_Y", "Total_X", "Total_Y"])
            writer.writerows(rear_mickey)
        self.get_logger().info(f"Saved Rear Mickey Data: {rear_mickey_path}")
        
        # 3. 車体キネマティクス計算 (移動距離・ステアリング角度)
        self.calculate_vehicle_kinematics(run_dir, front_mickey, rear_mickey, mickey_per_meter)

    def calculate_vehicle_kinematics(self, run_dir, front_mickey, rear_mickey, mickey_per_meter):
        """前輪・後輪のミッキーデータから車体の移動距離とステアリング角度を計算
        
        移動距離: 後輪の移動量（車体中心基準）
        ステアリング角度: 前後輪の移動量差からアッカーマンステアリングの式で計算
        
        アッカーマンステアリング: tan(steering_angle) = wheelbase / turning_radius
        前後輪の移動差と角速度から推定
        """
        if len(front_mickey) != len(rear_mickey):
            self.get_logger().error("Front and rear mickey data length mismatch")
            return
        
        kinematics_data = []
        total_distance = 0.0
        wheelbase_mickey = self.wheelbase * mickey_per_meter  # ホイールベース（ミッキー単位）
        
        for i in range(len(front_mickey)):
            timestamp = front_mickey[i][0]
            
            # 各輪の相対移動量（ミッキー）
            front_rel_x = front_mickey[i][1]
            front_rel_y = front_mickey[i][2]
            rear_rel_x = rear_mickey[i][1]
            rear_rel_y = rear_mickey[i][2]
            
            # 各輪の移動距離（ミッキー）
            front_dist = math.sqrt(front_rel_x**2 + front_rel_y**2)
            rear_dist = math.sqrt(rear_rel_x**2 + rear_rel_y**2)
            
            # 車体の移動距離 = 後輪の移動距離（車体中心の移動とほぼ同じ）
            # ミッキー -> メートル に変換
            distance_m = rear_dist / mickey_per_meter
            total_distance += distance_m
            
            # ステアリング角度の計算
            # 前後輪の移動差からヨーレート（角速度）を推定し、ステアリング角度を逆算
            # 
            # アッカーマンステアリングの関係:
            # 前輪と後輪の軌跡差 = wheelbase * sin(heading_change)
            # heading_change ≈ arc_diff / wheelbase (小角度近似)
            #
            # より厳密には:
            # steering_angle = atan2(wheelbase * yaw_rate, velocity)
            # yaw_rate ≈ (front_dist - rear_dist) / wheelbase
            
            if rear_dist > 0.01:  # ゼロ除算回避
                # 前後輪の速度差からヨーレートを推定
                yaw_rate_approx = (front_dist - rear_dist) / wheelbase_mickey
                
                # ステアリング角度 (ラジアン)
                # tan(steering) ≈ wheelbase * yaw_rate / velocity
                # ここでは簡略化: steering ≈ atan(yaw_rate * wheelbase / rear_dist)
                steering_rad = math.atan2(yaw_rate_approx * wheelbase_mickey, rear_dist)
            else:
                steering_rad = 0.0
            
            # 度に変換
            steering_deg = math.degrees(steering_rad)
            
            kinematics_data.append([
                timestamp,
                round(distance_m * 1000, 3),  # 移動距離 (mm)
                round(total_distance * 1000, 1),  # 累計移動距離 (mm)
                round(steering_deg, 2)  # ステアリング角度 (度)
            ])
        
        # CSVに保存
        kinematics_path = os.path.join(run_dir, "mickey_kinematics.csv")
        with open(kinematics_path, "w") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp_s", "Distance_mm", "Total_Distance_mm", "Steering_deg"])
            writer.writerows(kinematics_data)
        self.get_logger().info(f"Saved Vehicle Kinematics: {kinematics_path}")

    def generate_lidar_segments_mickey(self, run_dir):
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
        segments_path = os.path.join(run_dir, "mickey_lidar_segments.csv")
        with open(segments_path, "w") as f:
            writer = csv.writer(f)
            writer.writerow(["start_x", "start_y", "end_x", "end_y", "length", "angle"])
            writer.writerows(all_segments)
        self.get_logger().info(f"Saved Mickey-based Segments: {segments_path} ({len(all_segments)} segments)")


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
