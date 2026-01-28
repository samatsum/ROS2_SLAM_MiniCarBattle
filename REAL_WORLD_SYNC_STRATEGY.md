# 実機マウスセンサー（ミッキー）の非同期データ対策案

シミュレータと異なり、実機では2つのマウスセンサー（USBデバイス）からのデータ受信タイミングは完全に一致しません。また、PCの負荷やUSBのポーリングレートの揺らぎにより、ジッター（時間のズレ）が発生します。

これらをそのまま計算に使用すると、特にカーブ走行時に「前輪データは $t$ のものだが、後輪データは $t+5ms$ のもの」という不整合が起き、計算される角度 $\theta$ に大きな誤差が生じます。

## 解決策：線形補間によるリサンプリング

「それぞれのセンサーから来た生データ」を直接使うのではなく、**「ある決まった時刻 $t$ における推定位置」** を計算して揃える手法（リサンプリング）を推奨します。

### アルゴリズムの流れ

1.  **データバッファリング**:
    左右（または前後）のマウスから来た「積算カウント値」と「受信時刻」を、それぞれのリストに保存します。
    *   Front: `[(t1, x1, y1), (t2, x2, y2), ...]`
    *   Rear:  `[(t1', x1', y1'), (t2', x2', y2'), ...]`

2.  **マスタークロックでのトリガー**:
    メインループ（例えば 100Hz = 10msごと）で、現在の時刻 $t_{now}$ を基準に処理を行います。

3.  **線形補間 (Linear Interpolation)**:
    時刻 $t_{target}$ における前輪・後輪のカウント値を、前後のデータから推測します。

    $$
    X(t_{target}) = X_{prev} + \frac{X_{next} - X_{prev}}{t_{next} - t_{prev}} \times (t_{target} - t_{prev})
    $$

    これにより、**「完全に同じ時刻 $t_{target}$」** における Front と Rear の位置座標を得ることができます。

4.  **DR計算**:
    揃った Front/Rear の座標を使って、通常のDead Reckoning計算を行います。

### 実装イメージ (Python)

```python
class MickeySynchronizer:
    def __init__(self):
        self.front_buffer = [] # [(time, total_x, total_y), ...]
        self.rear_buffer = []  # [(time, total_x, total_y), ...]
        
    def add_data_front(self, timestamp, dx, dy):
        # 積算値をバッファに保存
        last_x = self.front_buffer[-1][1] if self.front_buffer else 0
        last_y = self.front_buffer[-1][2] if self.front_buffer else 0
        self.front_buffer.append((timestamp, last_x + dx, last_y + dy))
        
    def get_synchronized_state(self, target_time):
        # target_time 前後のデータを探して補間する
        f_x, f_y = self.interpolate(self.front_buffer, target_time)
        r_x, r_y = self.interpolate(self.rear_buffer, target_time)
        return f_x, f_y, r_x, r_y

    def interpolate(self, buffer, t):
        # (簡単のため探索ロジックは省略)
        # bufferの中から t を挟む2点を見つけ、内分点を返す
        ...
```

### 注意点

*   **遅延 (Latency)**: 補間を行うためには「$t_{target}$ より少し未来のデータ」が届いている必要があります。そのため、リアルタイム制御では数ミリ秒の遅延（バッファリング待ち）を許容するか、あるいは最新の速度ベクトルを使って「外挿（Extrapolation）」を行います。
    *   **後処理（ログ解析）の場合**: 全データが揃っているため、完璧な補間が可能です。
    *   **リアルタイム制御の場合**: 外挿を使うか、数サンプルの遅れを許容して補間します。

この手法により、ハードウェアレベルでの同期信号線などがなくても、ソフトウェア上で実用十分な同期精度を得ることができます。
