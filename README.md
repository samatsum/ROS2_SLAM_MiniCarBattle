# ROS 2 F1TENTH 自律走行スターターキット (初心者向けガイド)

このリポジトリは、自動運転車の大会「F1TENTH」のシミュレータ上で、障害物を回避しながら走行する「Follow the Gap (ギャップ・フォロー)」アルゴリズムを実装したプロジェクトです。

**ROS 2 (Robot Operating System 2)** のインストール方法も知らない、全くの初心者の方でも動かせるように、１から丁寧に手順を解説します。

---

## 🏎️ 何ができるの？

*   仮想空間上のF1カーをプログラムで自動運転させることができます。
*   LiDAR（レーザーセンサー）のデータを使って、壁にぶつからないように自動でハンドルを切ります。
*   走行した軌跡（通った道）を画像として保存できます。

---

## 💻 1. 準備するもの (推奨環境)

このプログラムを動かすには **Ubuntu (ウブントゥ)** というOSが必要です。

*   **PC**: 普通のノートPCやデスクトップPC
*   **OS**: **Ubuntu 22.04 LTS (Jammy Jellyfish)**
    *   ※ WindowsやMacの方は、「WSL2」や「VirtualBox」などを使ってUbuntu 22.04を用意してください。
*   **Python**: バージョン **3.10** (Ubuntu 22.04に標準で入っています)
    *   ※ 他のバージョンだと正常に動かない可能性があります。

---

## 🛠️ 2. 環境構築の手順

以下の手順を上から順番に、「ターミナル」（Windowsでいうコマンドプロンプト）にコピー＆ペーストして実行してください。
ターミナルは `Ctrl + Alt + T` で開けます。

### Step 1: ROS 2 (Humble) のインストール

ロボットを動かすための基本ソフト「ROS 2 Humble」を入れます。

```bash
# 1. ロケール（言語設定）の確認と設定
locale  # UTF-8になっているか確認

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. 必要なツールのインストール
sudo apt install software-properties-common
sudo add-apt-repository universe

# 3. ROS 2のリポジトリキーを追加
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 4. リポジトリをソースリストに追加
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 5. ROS 2のインストール (時間がかかります☕)
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop -y

# 6. 環境変数の設定 (ターミナルを開くたびにROS2を使えるようにする)
# まだ設定していない場合のみ追記
grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc || echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 7. 追加ツールのインストール (ビルドツールなど)
sudo apt install python3-colcon-common-extensions python3-rosdep -y
```

ここまでエラーが出ずに終われば、ROS 2の準備は完了です！🎉

### Step 2: ワークスペースの作成とシミュレータの準備

プログラムを置く場所（ワークスペース）を作り、F1TENTHシミュレータをダウンロードします。

```bash
# 1. rosdepの初期化
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# 2. ワークスペースフォルダの作成
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws/src

# 3. F1TENTHシミュレータのダウンロード
git clone https://github.com/f1tenth/f1tenth_gym_ros.git

# 4. この自律走行プログラムのダウンロード
git clone https://github.com/samatsum/ROS2_SLAM_MiniCarBattle.git
```

### Step 3: 依存ライブラリのインストールとビルド

必要な外部ライブラリを自動でインストールし、プログラムを組み立て（ビルド）ます。

```bash
# 1. ワークスペースのルートに戻る
cd ~/f1tenth_ws

# 2. 依存関係のインストール
rosdep install -i --from-path src --rosdistro humble -y

# 3. ビルド実行 (これもしばらくかかります☕)
colcon build

# 4. ビルドした機能を有効化
source install/setup.bash
echo "source ~/f1tenth_ws/install/setup.bash" >> ~/.bashrc
```

---

## 🚀 3. 実行方法

準備お疲れ様でした！いよいよ動かします。

### シミュレーションの開始

新しいターミナルを開き、以下のコマンドを入力します。

```bash
ros2 launch gap_follow gap_follow.launch.py
```

**成功すると…**
1.  **RViz (アールビズ)** という画面が立ち上がります。
2.  画面の中にF1カーとコースが表示されます。
3.  車が自動的に走り出し、壁を避けながら周回します！🏎️💨

### ログデータの確認

走行が終わったら（`Ctrl + C` で終了）、以下のフォルダにログや画像が保存されています。

`~/f1tenth_ws/log/courseLog/YYYYMMDD_HHMMSS_manual/images/`

ここに、LiDARが捉えたマップや走行軌跡の画像 (`kinematics.png`) が生成されています。

---

## ⚙️ 4. 設定の変更

車の速度や、壁との距離などを調整したい場合は、以下のファイルを編集します。

`~/f1tenth_ws/src/ROS2_SLAM_MiniCarBattle/src/gap_follow/launch/gap_follow.launch.py`

```python
        parameters=[
            {"velocity": 2.5},      # 速度 (m/s) - 数字を大きくすると速くなる
            {"target_dist": 0.8},   # 壁との目標距離
            {"kp": 1.5},            # ハンドルの切りやすさ (大きくすると敏感になる)
        ],
```

編集後は、必ず再度ビルドを行ってください：
```bash
cd ~/f1tenth_ws
colcon build
source install/setup.bash
```

---

## ❓ よくあるトラブル

**Q. `ros2: command not found` と言われる**
A. `source ~/.bashrc` を実行するか、ターミナルを開き直してみてください。

**Q. ビルドでエラーが出る**
A. `rosdep install ...` の手順を飛ばしていませんか？ 必要なライブラリが足りていない可能性があります。

**Q. 車が動かない**
A. RVizの画面左下の「Reset」ボタンを押してみるか、もう一度 `ros2 launch ...` を実行し直してみてください。
