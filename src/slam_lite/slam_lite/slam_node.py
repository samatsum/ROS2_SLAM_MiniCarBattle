#!/usr/bin/env python3
"""
SLAM Lite Node
軽量SLAMのメインノード
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid as OccupancyGridMsg
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
import math
from transforms3d.euler import quat2euler, euler2quat

from slam_lite.occupancy_grid import OccupancyGrid
from slam_lite.scan_matcher import ScanMatcher
from slam_lite.line_extractor import LineExtractor


class SlamLiteNode(Node):
    """軽量SLAMノード"""
    
    def __init__(self):
        super().__init__('slam_lite_node')
        
        # パラメータ
        self.declare_parameter('map_size', 20.0)      # マップサイズ (m)
        self.declare_parameter('resolution', 0.05)    # マップ解像度 (m/cell)
        self.declare_parameter('use_odom', True)      # オドメトリを使用するか
        self.declare_parameter('publish_rate', 2.0)   # マップ配信レート (Hz)
        self.declare_parameter('save_segments', True) # 線分をCSV保存するか
        
        map_size = self.get_parameter('map_size').value
        resolution = self.get_parameter('resolution').value
        
        # 占有グリッドマップ
        self.grid_map = OccupancyGrid(
            width_m=map_size,
            height_m=map_size,
            resolution=resolution
        )
        
        # スキャンマッチャー
        self.scan_matcher = ScanMatcher(max_iterations=15, tolerance=0.001)
        
        # 線分抽出器 (0.07m閾値)
        self.line_extractor = LineExtractor(
            distance_threshold=0.07,
            min_points=3,
            min_length=0.1
        )
        
        # ロボットの位置 (x, y, theta)
        self.robot_pose = [0.0, 0.0, 0.0]
        
        # オドメトリ情報
        self.last_odom = None
        self.odom_delta = [0.0, 0.0, 0.0]  # 前回からの移動量
        
        # 線分統計
        self.total_segments = 0
        self.scan_count = 0
        
        # Subscribers
        # スキャントピック (パラメータで変更可能)
        self.declare_parameter('scan_topic', '/scan')
        scan_topic = self.get_parameter('scan_topic').value
        self.get_logger().info(f'Subscribing to scan topic: {scan_topic}')
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )
        
        # Publishers
        self.map_pub = self.create_publisher(
            OccupancyGridMsg,
            '/slam_lite/map',
            10
        )
        
        self.pose_pub = self.create_publisher(
            Marker,
            '/slam_lite/pose',
            10
        )
        
        self.segments_pub = self.create_publisher(
            MarkerArray,
            '/slam_lite/segments',
            10
        )
        
        # マップ配信タイマー
        publish_rate = self.get_parameter('publish_rate').value
        self.map_timer = self.create_timer(1.0 / publish_rate, self.publish_map)
        
        self.get_logger().info('SLAM Lite Node Started!')
        self.get_logger().info(f'Map: {map_size}m x {map_size}m, resolution: {resolution}m')
        self.get_logger().info('Line Extractor: threshold=0.07m')
    
    def odom_callback(self, msg):
        """オドメトリコールバック"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        _, _, theta = quat2euler([quat.w, quat.x, quat.y, quat.z], axes='sxyz')
        
        if self.last_odom is not None:
            dx = x - self.last_odom[0]
            dy = y - self.last_odom[1]
            dtheta = theta - self.last_odom[2]
            
            while dtheta > math.pi:
                dtheta -= 2 * math.pi
            while dtheta < -math.pi:
                dtheta += 2 * math.pi
            
            self.odom_delta = [dx, dy, dtheta]
        
        self.last_odom = [x, y, theta]
    
    def scan_callback(self, msg):
        """LIDARスキャンコールバック"""
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # ポイントクラウドに変換
        points = self.scan_matcher.scan_to_points(ranges, angles)
        
        # デバッグ: 最初の数回は詳細ログ
        if self.scan_count < 5:
            self.get_logger().info(f'[DEBUG] Scan received: {len(ranges)} ranges, {len(points)} valid points')
        
        if len(points) < 10:
            if self.scan_count < 5:
                self.get_logger().warn(f'[DEBUG] Not enough points: {len(points)} < 10')
            return
        
        # 線分抽出（ロボット座標系）
        segments_robot = self.line_extractor.extract_segments(points)
        
        # デバッグ: 最初の数回は詳細ログ
        if self.scan_count < 5:
            self.get_logger().info(f'[DEBUG] Extracted {len(segments_robot)} segments from {len(points)} points')
        
        # ワールド座標系に変換
        segments_world = self.line_extractor.transform_segments(
            segments_robot,
            self.robot_pose[0],
            self.robot_pose[1],
            self.robot_pose[2]
        )
        
        # 基準線分として保存
        self.line_extractor.save_reference_segments(segments_world)
        
        # 統計更新
        self.total_segments += len(segments_world)
        self.scan_count += 1
        
        # 10スキャンごとにログ出力
        if self.scan_count % 10 == 0:
            avg = self.total_segments / self.scan_count
            ref_count = len(self.line_extractor.reference_segments)
            self.get_logger().info(f'Scan #{self.scan_count}: {len(segments_world)} segs, total ref: {ref_count}')
        
        # オドメトリからの初期推定
        use_odom = self.get_parameter('use_odom').value
        if use_odom and self.last_odom is not None:
            initial_guess = tuple(self.odom_delta)
        else:
            initial_guess = (0, 0, 0)
        
        # スキャンマッチング
        (dx, dy, dtheta), success = self.scan_matcher.match(points, initial_guess)
        
        if success:
            c = math.cos(self.robot_pose[2])
            s = math.sin(self.robot_pose[2])
            
            self.robot_pose[0] += c * dx - s * dy
            self.robot_pose[1] += s * dx + c * dy
            self.robot_pose[2] += dtheta
            
            while self.robot_pose[2] > math.pi:
                self.robot_pose[2] -= 2 * math.pi
            while self.robot_pose[2] < -math.pi:
                self.robot_pose[2] += 2 * math.pi
        
        # マップを更新
        self.grid_map.update_from_scan(
            self.robot_pose[0],
            self.robot_pose[1],
            self.robot_pose[2],
            ranges,
            angles
        )
        
        # 可視化
        self.publish_pose()
        self.publish_segments(segments_world)
    
    def publish_map(self):
        """占有グリッドマップを配信"""
        msg = OccupancyGridMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.info.resolution = self.grid_map.resolution
        msg.info.width = self.grid_map.width
        msg.info.height = self.grid_map.height
        
        msg.info.origin.position.x = self.grid_map.origin_x
        msg.info.origin.position.y = self.grid_map.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        prob = self.grid_map.get_probability_map()
        unknown_mask = np.abs(self.grid_map.grid) < 0.1
        
        ros_map = np.zeros_like(prob, dtype=np.int8)
        ros_map[~unknown_mask] = (prob[~unknown_mask] * 100).astype(np.int8)
        ros_map[unknown_mask] = -1
        
        msg.data = ros_map.flatten().tolist()
        
        self.map_pub.publish(msg)
    
    def publish_pose(self):
        """ロボットポーズをマーカーとして配信"""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'slam_pose'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        marker.pose.position.x = self.robot_pose[0]
        marker.pose.position.y = self.robot_pose[1]
        marker.pose.position.z = 0.1
        
        quat = euler2quat(0, 0, self.robot_pose[2], axes='sxyz')
        marker.pose.orientation.w = quat[0]
        marker.pose.orientation.x = quat[1]
        marker.pose.orientation.y = quat[2]
        marker.pose.orientation.z = quat[3]
        
        marker.scale.x = 0.5
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.pose_pub.publish(marker)
    
    def publish_segments(self, segments):
        """線分をマーカーとして配信"""
        marker_array = MarkerArray()
        
        for i, seg in enumerate(segments):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'map'
            marker.ns = 'segments'
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # 線分の始点と終点
            p1 = Point()
            p1.x = seg.start[0]
            p1.y = seg.start[1]
            p1.z = 0.05
            
            p2 = Point()
            p2.x = seg.end[0]
            p2.y = seg.end[1]
            p2.z = 0.05
            
            marker.points = [p1, p2]
            
            marker.scale.x = 0.03  # 線の太さ
            
            # 青色
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 1.0
            marker.color.a = 1.0
            
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 200000000  # 0.2秒
            
            marker_array.markers.append(marker)
        
        self.segments_pub.publish(marker_array)
    
    def save_current_segments(self):
        """現在の基準線分をCSVに保存"""
        save_segments = self.get_parameter('save_segments').value
        if save_segments:
            filepath = self.line_extractor.save_segments_to_csv(
                self.line_extractor.reference_segments
            )
            self.get_logger().info(f'Saved segments: {filepath}')


# グローバル変数（シグナルハンドラ用）
_node = None
_segments_saved = False  # 二重保存防止フラグ


def save_segments_once():
    """セグメントを一度だけ保存する"""
    global _node, _segments_saved
    if _node is not None and not _segments_saved:
        try:
            _node.get_logger().warn("Saving segments before exit...")
            print("SLAM_LITE: Saving segments before exit...")
            _node.save_current_segments()
            _segments_saved = True
        except Exception as e:
            print(f"SLAM_LITE: Error saving segments: {e}")
            import traceback
            traceback.print_exc()


def signal_handler(sig, frame):
    """Ctrl+C が押された時のハンドラ"""
    save_segments_once()
    rclpy.shutdown()


def on_shutdown_callback():
    """ROS 2 コンテキストのシャットダウンコールバック"""
    save_segments_once()


def main(args=None):
    global _node
    import signal
    
    rclpy.init(args=args)
    
    # on_shutdown コールバックを登録（launchで起動された場合もこれで保存される）
    context = rclpy.get_default_context()
    context.on_shutdown(on_shutdown_callback)
    
    _node = SlamLiteNode()
    
    # Ctrl+C のシグナルハンドラも登録（直接実行された場合のフォールバック）
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)  # SIGTERMも処理
    
    try:
        rclpy.spin(_node)
    except Exception:
        pass
    finally:
        # finally節でも保存を試みる（追加の安全策）
        save_segments_once()
        if _node is not None:
            _node.destroy_node()


if __name__ == '__main__':
    main()
