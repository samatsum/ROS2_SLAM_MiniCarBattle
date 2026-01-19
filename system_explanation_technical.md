# システム技術解説書 (Technical Explanation)

本ドキュメントでは、F1TENTH向けログ記録・解析システムのアーキテクチャとデータフローについて、技術的な観点から解説します。

## 1. システム・アーキテクチャ概要

本システムは分散したROS 2ノード群によって構成され、非同期に発生するセンサーデータの同期記録と、特徴量抽出による軽量ログの生成を行っています。

### 主要コンポーネント (Nodes)

1.  **`logger_node` (TrajectoryLogger)**
    *   **責務**: システム全体のログ管理、高周波オドメトリ処理、画像生成。
    *   **入力**:
        *   `/ego_racecar/odom` (Odometry): 車両の推定位相（Ground Truthとして利用）。
        *   Mickey Sensor Data (Raw): 独自実装の光学センサーインターフェース（500Hzポーリング）。
    *   **出力**:
        *   `mickey/`: Mickeyセンサー由来の生データおよびキネマティクス計算結果。
        *   `simulation/simulation_position.csv`: 比較基準となるOdomデータ。
        *   `images/`: 走行軌跡の可視化画像。

2.  **`slam_node` (SlamLiteNode)**
    *   **責務**: LIDARデータの処理、特徴量抽出 (Line Feature Extraction)。
    *   **入力**:
        *   `/scan` (LaserScan): 2D LIDAR点群データ。
    *   **出力**:
        *   `simulation/simulation_lidar_segments.csv`: 抽出された線分特徴量。

---

## 2. データ処理パイプラインの詳細

### A. Mickey Kinematics (独自オドメトリ生成)
通常のエンコーダやIMUに加え、光学式センサー（Mickey）を用いて車両の微細な挙動を計測しています。

*   **処理ロジック**:
    *   フロント/リアの各センサーから `(dx, dy)` の変位量を取得。
    *   **Ackermann Kinematics Model** に基づき、前後輪の移動差分からステアリング角、スリップ角、および `yaw` レートを推定。
    *   これを積分し、独自のローカル座標系におけるオドメトリ（Mickey Odom）を構築。
*   **目的**:
    *   スリップ（ドリフト）発生時の挙動解析。標準Odom（Simulation）との偏差比較によるモデル評価。

### B. Feature Extraction (線分抽出)
生の大容量点群データ（PointCloud）をそのまま保存するのではなく、環境地図作成に必要な「幾何学的特徴」のみを抽出・軽量化しています。

*   **処理ロジック (LineExtractor)**:
    *   `/scan` トピックのレンジデータを解析。
    *   **Split-and-Merge** アルゴリズム（または類似のセグメンテーション手法）を用い、連続する点群を「線分 (Line Segment)」に近似。
    *   各線分は `(start_x, start_y, end_x, end_y)` のベクトルとして表現される。
*   **座標変換 (Coordinate Transformation)**:
    *   抽出された線分は、そのフレーム取得時点での車両ポーズ（Mickey Odom または Simulation Odom）に基づき、グローバル座標系（Start地点基準）へ変換されて保存される。

---

## 3. ログ同期と永続化 (Synchronization & Persistence)

ROS 2の非同期なノード間で、同一の「Run（走行）」ディレクトリにデータを集約するための仕組みを実装しています。

1.  **ディレクトリ構造の正規化**:
    *   `logger_node` がトリガー（終了時またはCommand）となり、タイムスタンプ付きディレクトリ (`log/courseLog/YYYYMMDD_.../`) を生成。
    *   **Hierarchical Naming**: ファイル名には親ディレクトリのコンテキストを含める (`mickey_front.csv`, `simulation_position.csv`) ことで、データソースの混同を防ぐ。

2.  **レースコンディションの解決**:
    *   `slam_node` は終了時 (`SIGINT` ハンドリング時)、`logger_node` がディレクトリ作成を完了するのを待機（ポーリング）するロジックを持つ。これにより、保存先ディレクトリ未作成によるデータ欠損を防いでいる。

## 4. データの活用 (Application)

生成されたCSV群は、オフラインでの後処理に最適化されています。

*   **Scan Matching / SLAM**:
    *   `lidar_segments` データを用いた特徴ベースのSLAM（Graph SLAM等）の入力として利用可能。点群マッチング（ICP）に比べ計算コストが低い。
*   **Sensor Fusion Evaluation**:
    *   `mickey_kinematics` と `simulation_position` の軌跡比較により、実環境におけるタイヤの滑り率やオドメトリの誤差モデルを同定できる。
