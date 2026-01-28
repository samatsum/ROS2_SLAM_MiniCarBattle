# 🏎️ よく使うコマンドリスト (Cheat Sheet)

[English](COMMANDS.md)

環境構築が完了した後に、日々の開発でよく使うコマンドをまとめました。

## 1. シミュレーションの実行

全ての基本となるコマンドです。 **2つのターミナル** を使います。

**ターミナル 1 (シミュレータのみ):**
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

**ターミナル 2 (自動運転):**
```bash
ros2 launch gap_follow gap_follow.launch.py
```

## 2. 設定変更後の再ビルド

`gap_follow.launch.py` や `reactive_node.cpp` を編集した後は、反映させるためにビルドが必要です。
全体をビルドすると遅いので、変更したパッケージだけを指定して高速にビルドします。

```bash
cd ~/f1tenth_ws
colcon build --packages-select gap_follow --symlink-install
source install/setup.bash
```

*   `--packages-select gap_follow`: このパッケージだけビルドする
*   `--symlink-install`: Pythonスクリプトの変更をビルドなしで即反映させるオプション（C++の変更には無効ですが、入れておくと便利です）

## 3. ログの確認

生成されたログ画像のディレクトリをエクスプローラー（ファイルマネージャ）で開きます。

```bash
xdg-open ~/f1tenth_ws/log/courseLog/
```

最新のログディレクトリに移動して画像を確認する場合：

```bash
cd ~/f1tenth_ws/log/courseLog/$(ls -t ~/f1tenth_ws/log/courseLog/ | head -n 1)/images
eog .  # 画像ビューアを開く
```

## 4. トラブルシューティング

### シミュレータが固まった時や、おかしい時
一度全てのROS 2プロセスを強制終了します。

```bash
pkill -f ros
```

その後に再度 `ros2 launch ...` を実行してください。

### 実行中のノードを確認する

```bash
ros2 node list
```

### 実行中のトピック（データのやり取り）を確認する

```bash
ros2 topic list
```

### 特定のトピックの中身を覗き見る（例：LiDARデータ）

```bash
ros2 topic echo /scan
```
※ データの流れが速すぎて見えないので `Ctrl + C` で止めます。
