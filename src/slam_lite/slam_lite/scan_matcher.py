"""
Scan Matcher Module
シンプルなICP (Iterative Closest Point) 実装
"""
import numpy as np
import math


class ScanMatcher:
    """シンプルなスキャンマッチャー
    
    2つのポイントクラウド間の変換を推定する
    """
    
    def __init__(self, max_iterations=20, tolerance=0.001):
        """
        Args:
            max_iterations: ICPの最大反復回数
            tolerance: 収束判定の閾値
        """
        self.max_iterations = max_iterations
        self.tolerance = tolerance
        self.prev_scan = None
    
    def scan_to_points(self, ranges, angles):
        """LIDARスキャンをポイントクラウドに変換
        
        Args:
            ranges: 距離の配列
            angles: 角度の配列
        
        Returns:
            numpy.ndarray: (N, 2) の点群
        """
        valid = np.isfinite(ranges) & (ranges > 0.01)
        r = ranges[valid]
        a = angles[valid]
        
        x = r * np.cos(a)
        y = r * np.sin(a)
        
        return np.column_stack([x, y])
    
    def match(self, current_scan, initial_guess=(0, 0, 0)):
        """現在のスキャンを前回のスキャンにマッチング
        
        Args:
            current_scan: (N, 2) のポイントクラウド
            initial_guess: 初期推定 (dx, dy, dtheta)
        
        Returns:
            (dx, dy, dtheta): 推定された変換
            success: マッチングが成功したか
        """
        if self.prev_scan is None or len(self.prev_scan) < 10:
            self.prev_scan = current_scan
            return (0, 0, 0), False
        
        if len(current_scan) < 10:
            return initial_guess, False
        
        # ICPで変換を推定
        dx, dy, dtheta = initial_guess
        
        for i in range(self.max_iterations):
            # 現在のスキャンを変換
            transformed = self._transform_points(current_scan, dx, dy, dtheta)
            
            # 最近傍点を探索
            correspondences = self._find_correspondences(transformed, self.prev_scan)
            
            if len(correspondences) < 5:
                break
            
            # 変換を更新
            src_pts = current_scan[correspondences[:, 0]]
            tgt_pts = self.prev_scan[correspondences[:, 1]]
            
            new_dx, new_dy, new_dtheta = self._estimate_transform(src_pts, tgt_pts)
            
            # 収束チェック
            delta = abs(new_dx - dx) + abs(new_dy - dy) + abs(new_dtheta - dtheta)
            dx, dy, dtheta = new_dx, new_dy, new_dtheta
            
            if delta < self.tolerance:
                break
        
        # 前回のスキャンを更新
        self.prev_scan = current_scan
        
        return (dx, dy, dtheta), True
    
    def _transform_points(self, points, dx, dy, dtheta):
        """点群を変換"""
        c = math.cos(dtheta)
        s = math.sin(dtheta)
        
        transformed = np.zeros_like(points)
        transformed[:, 0] = c * points[:, 0] - s * points[:, 1] + dx
        transformed[:, 1] = s * points[:, 0] + c * points[:, 1] + dy
        
        return transformed
    
    def _find_correspondences(self, src, tgt, max_dist=0.5):
        """最近傍点を探索（シンプルな全探索）"""
        correspondences = []
        
        for i, p in enumerate(src):
            # 各点について最近傍を探す
            dists = np.sqrt(np.sum((tgt - p)**2, axis=1))
            min_idx = np.argmin(dists)
            
            if dists[min_idx] < max_dist:
                correspondences.append([i, min_idx])
        
        return np.array(correspondences) if correspondences else np.array([]).reshape(0, 2)
    
    def _estimate_transform(self, src, tgt):
        """対応点から変換を推定（SVD法）"""
        # 重心
        src_mean = np.mean(src, axis=0)
        tgt_mean = np.mean(tgt, axis=0)
        
        # 中心化
        src_centered = src - src_mean
        tgt_centered = tgt - tgt_mean
        
        # 共分散行列
        H = src_centered.T @ tgt_centered
        
        # SVD
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # 反射を避ける
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # 回転角度
        theta = math.atan2(R[1, 0], R[0, 0])
        
        # 並進
        t = tgt_mean - R @ src_mean
        
        return t[0], t[1], theta
    
    def reset(self):
        """スキャンマッチャーをリセット"""
        self.prev_scan = None
