"""
Line Extractor Module
点群から線分を抽出し、周回比較用に保存
"""
import numpy as np
import math
import os
import csv
from datetime import datetime


class LineSegment:
    """線分を表すクラス"""
    
    def __init__(self, start_x, start_y, end_x, end_y):
        self.start = np.array([start_x, start_y])
        self.end = np.array([end_x, end_y])
    
    @property
    def length(self):
        return np.linalg.norm(self.end - self.start)
    
    @property
    def angle(self):
        """線分の角度 (ラジアン)"""
        diff = self.end - self.start
        return math.atan2(diff[1], diff[0])
    
    @property
    def midpoint(self):
        return (self.start + self.end) / 2
    
    def to_dict(self):
        return {
            'start_x': self.start[0],
            'start_y': self.start[1],
            'end_x': self.end[0],
            'end_y': self.end[1],
            'length': self.length,
            'angle': self.angle
        }


class LineExtractor:
    """点群から線分を抽出するクラス"""
    
    def __init__(self, distance_threshold=0.07, min_points=3, min_length=0.1):
        """
        Args:
            distance_threshold: 隣接点の最大距離 (m) - これを超えると別の線分
            min_points: 線分を構成する最小点数
            min_length: 線分の最小長さ (m)
        """
        self.distance_threshold = distance_threshold
        self.min_points = min_points
        self.min_length = min_length
        
        # 基準線分の保存用
        self.reference_segments = []
        self.lap_count = 0
        
        # ログ保存ディレクトリ (courseLogと同じ形式)
        self.log_base_dir = os.path.expanduser("~/f1tenth_ws/log/courseLog")
        self.run_dir = None  # 実行時に設定される
        
    def set_run_dir(self, run_dir):
        """ログ保存先ディレクトリを設定"""
        self.run_dir = run_dir
        if not os.path.exists(self.run_dir):
            os.makedirs(self.run_dir)
    
    def find_latest_run_dir(self):
        """logger_nodeが作成した最新のディレクトリを探す"""
        if not os.path.exists(self.log_base_dir):
            os.makedirs(self.log_base_dir)
            return None
        
        # _manual または _crash で終わるディレクトリを探す
        dirs = []
        for d in os.listdir(self.log_base_dir):
            full_path = os.path.join(self.log_base_dir, d)
            if os.path.isdir(full_path) and ('_manual' in d or '_crash' in d):
                dirs.append((full_path, os.path.getmtime(full_path)))
        
        if not dirs:
            return None
        
        # 最新のディレクトリを返す
        dirs.sort(key=lambda x: x[1], reverse=True)
        return dirs[0][0]
    
    def create_run_dir(self):
        """新しいログディレクトリを作成 (フォールバック用)"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(self.log_base_dir, f"{timestamp}_slam")
        if not os.path.exists(self.run_dir):
            os.makedirs(self.run_dir)
        return self.run_dir
    
    def extract_segments(self, points):
        """点群から線分を抽出
        
        Args:
            points: (N, 2) のポイントクラウド
        
        Returns:
            list[LineSegment]: 抽出された線分のリスト
        """
        if len(points) < self.min_points:
            return []
        
        segments = []
        current_group = [points[0]]
        
        for i in range(1, len(points)):
            # 隣接点との距離を計算
            dist = np.linalg.norm(points[i] - points[i-1])
            
            if dist <= self.distance_threshold:
                # 同じグループに追加
                current_group.append(points[i])
            else:
                # 新しいグループを開始
                segment = self._fit_line(current_group)
                if segment is not None:
                    segments.append(segment)
                current_group = [points[i]]
        
        # 最後のグループを処理
        segment = self._fit_line(current_group)
        if segment is not None:
            segments.append(segment)
        
        return segments
    
    def _fit_line(self, points):
        """点群に線分をフィット
        
        Args:
            points: 点のリスト
        
        Returns:
            LineSegment or None
        """
        if len(points) < self.min_points:
            return None
        
        pts = np.array(points)
        
        # 始点と終点
        start = pts[0]
        end = pts[-1]
        
        # 長さチェック
        length = np.linalg.norm(end - start)
        if length < self.min_length:
            return None
        
        return LineSegment(start[0], start[1], end[0], end[1])
    
    def transform_segments(self, segments, robot_x, robot_y, robot_theta):
        """線分をロボット座標系からワールド座標系に変換
        
        Args:
            segments: ロボット座標系での線分リスト
            robot_x, robot_y, robot_theta: ロボットの位置と向き
        
        Returns:
            list[LineSegment]: ワールド座標系での線分
        """
        c = math.cos(robot_theta)
        s = math.sin(robot_theta)
        
        world_segments = []
        for seg in segments:
            # 始点を変換
            wx1 = c * seg.start[0] - s * seg.start[1] + robot_x
            wy1 = s * seg.start[0] + c * seg.start[1] + robot_y
            
            # 終点を変換
            wx2 = c * seg.end[0] - s * seg.end[1] + robot_x
            wy2 = s * seg.end[0] + c * seg.end[1] + robot_y
            
            world_segments.append(LineSegment(wx1, wy1, wx2, wy2))
        
        return world_segments
    
    def save_reference_segments(self, segments):
        """基準線分としてメモリに保存（1周目用）"""
        self.reference_segments.extend(segments)
    
    def _wait_for_run_dir(self, timeout=3.0):
        """最新のログディレクトリが見つかるまで待機"""
        import time
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            latest = self.find_latest_run_dir()
            if latest:
                # 直近（60秒以内）に作成/更新されたものか確認
                try:
                    mtime = os.path.getmtime(latest)
                    if (datetime.now().timestamp() - mtime) < 60:
                        return latest
                except OSError:
                    pass
            time.sleep(0.5)
            
        # 見つからなければ新規作成
        return self.create_run_dir()
    
    def save_segments_to_csv(self, segments, lap_number=None):
        """線分をCSVファイルに保存
        
        Args:
            segments: 保存する線分リスト
            lap_number: 周回番号
        """
        if lap_number is None:
            lap_number = self.lap_count
        
        # run_dirがなければlogger_nodeが作成した最新ディレクトリを探す（待機付き）
        if self.run_dir is None:
            self.run_dir = self._wait_for_run_dir()
        
        # simulation/サブディレクトリに保存
        simulation_dir = os.path.join(self.run_dir, "simulation")
        if not os.path.exists(simulation_dir):
            os.makedirs(simulation_dir)
        
        filename = "simulation_lidar_segments.csv"
        filepath = os.path.join(simulation_dir, filename)
        
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['start_x', 'start_y', 'end_x', 'end_y', 'length', 'angle'])
            
            for seg in segments:
                writer.writerow([
                    seg.start[0], seg.start[1],
                    seg.end[0], seg.end[1],
                    seg.length, seg.angle
                ])
        
        return filepath
    
    def compare_with_reference(self, current_segments, threshold=0.5):
        """現在の線分を基準線分と比較
        
        Args:
            current_segments: 現在のスキャンから抽出した線分
            threshold: マッチング閾値 (m)
        
        Returns:
            match_score: 0.0〜1.0のマッチングスコア
            matches: マッチした線分のペアリスト
        """
        if not self.reference_segments or not current_segments:
            return 0.0, []
        
        matches = []
        
        for curr_seg in current_segments:
            best_dist = float('inf')
            best_ref = None
            
            for ref_seg in self.reference_segments:
                # 中点間の距離で比較
                dist = np.linalg.norm(curr_seg.midpoint - ref_seg.midpoint)
                
                if dist < best_dist and dist < threshold:
                    best_dist = dist
                    best_ref = ref_seg
            
            if best_ref is not None:
                matches.append((curr_seg, best_ref, best_dist))
        
        match_score = len(matches) / max(len(current_segments), 1)
        
        return match_score, matches
    
    def increment_lap(self):
        """周回カウントをインクリメント"""
        self.lap_count += 1
    
    def clear_reference(self):
        """基準線分をクリア"""
        self.reference_segments = []
