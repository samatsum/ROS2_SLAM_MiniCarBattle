"""
Occupancy Grid Map Module
軽量な占有グリッドマップの実装
"""
import numpy as np
import math


class OccupancyGrid:
    """占有グリッドマップ
    
    各セルは対数オッズで確率を管理:
    - 0: 不明 (確率50%)
    - 正: 障害物がある可能性が高い
    - 負: 空いている可能性が高い
    """
    
    def __init__(self, width_m=20.0, height_m=20.0, resolution=0.05):
        """
        Args:
            width_m: マップの幅 (メートル)
            height_m: マップの高さ (メートル)
            resolution: 1セルあたりのメートル
        """
        self.resolution = resolution
        self.width = int(width_m / resolution)
        self.height = int(height_m / resolution)
        
        # マップ原点（中央）のワールド座標
        self.origin_x = -width_m / 2.0
        self.origin_y = -height_m / 2.0
        
        # 対数オッズグリッド (0 = 不明)
        self.grid = np.zeros((self.height, self.width), dtype=np.float32)
        
        # 対数オッズの更新パラメータ
        self.l_occ = 0.85   # 障害物検出時の増分
        self.l_free = -0.4  # 空き検出時の減分
        self.l_max = 3.5    # 対数オッズの上限
        self.l_min = -2.0   # 対数オッズの下限
    
    def world_to_grid(self, x, y):
        """ワールド座標 -> グリッド座標"""
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy
    
    def grid_to_world(self, gx, gy):
        """グリッド座標 -> ワールド座標"""
        x = gx * self.resolution + self.origin_x
        y = gy * self.resolution + self.origin_y
        return x, y
    
    def is_valid(self, gx, gy):
        """グリッド座標が有効かチェック"""
        return 0 <= gx < self.width and 0 <= gy < self.height
    
    def update_from_scan(self, robot_x, robot_y, robot_theta, ranges, angles):
        """LIDARスキャンからマップを更新
        
        Args:
            robot_x, robot_y, robot_theta: ロボットの位置と向き
            ranges: 距離の配列
            angles: 角度の配列 (ロボット座標系)
        """
        robot_gx, robot_gy = self.world_to_grid(robot_x, robot_y)
        
        for i, (r, angle) in enumerate(zip(ranges, angles)):
            if not np.isfinite(r) or r <= 0:
                continue
            
            # 障害物の位置（ワールド座標）
            world_angle = robot_theta + angle
            hit_x = robot_x + r * math.cos(world_angle)
            hit_y = robot_y + r * math.sin(world_angle)
            hit_gx, hit_gy = self.world_to_grid(hit_x, hit_y)
            
            # ブレゼンハムのアルゴリズムでレイキャスト
            self._ray_cast(robot_gx, robot_gy, hit_gx, hit_gy)
    
    def _ray_cast(self, x0, y0, x1, y1):
        """ブレゼンハムのアルゴリズムでレイトレース
        
        始点から終点の手前まで: 空き
        終点: 障害物
        """
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            if self.is_valid(x, y):
                if x == x1 and y == y1:
                    # 終点 = 障害物
                    self.grid[y, x] = min(self.grid[y, x] + self.l_occ, self.l_max)
                else:
                    # 途中 = 空き
                    self.grid[y, x] = max(self.grid[y, x] + self.l_free, self.l_min)
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    
    def get_probability_map(self):
        """対数オッズを確率 [0, 1] に変換"""
        return 1.0 - 1.0 / (1.0 + np.exp(self.grid))
    
    def get_image(self):
        """可視化用の画像を生成 (0-255)
        
        Returns:
            numpy.ndarray: グレースケール画像 (0=空き, 127=不明, 255=障害物)
        """
        prob = self.get_probability_map()
        img = (prob * 255).astype(np.uint8)
        return img
