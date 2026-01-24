#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <cmath>
#include <vector>
#include <algorithm>
#include <string>

class ReactiveGapFollower : public rclcpp::Node {
public:
    ReactiveGapFollower() : Node("reactive_node") {
        // Parameters
        this->declare_parameter("bubble_radius", 1.0);      // 障害物回避バブルの半径 (m)
        this->declare_parameter("preprocess_conv_size", 3); // ノイズ除去のための平均化ウィンドウサイズ
        this->declare_parameter("max_lidar_dist", 3.0);     // これ以上遠い点は「フリースペース」とみなしてクリップする距離 (m)
        this->declare_parameter("field_of_view_deg", 180.0); // 使用するLiDARの視野角 (前方中心に左右)
        this->declare_parameter("speed", 1.5);              // 基本速度

        // Publishers & Subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ReactiveGapFollower::scan_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);
            
        RCLCPP_INFO(this->get_logger(), "Reactive Gap Follower Node Started");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    // 受け取ったスキャンデータを前処理する
    void preprocess_lidar(std::vector<double>& ranges, double range_max) {
        // 1. NaN/Inf の除去 と 最大距離のクリップ
        double max_dist = this->get_parameter("max_lidar_dist").as_double();
        
        for (double &r : ranges) {
            if (std::isnan(r) || std::isinf(r)) {
                r = 0.0;
            } else if (r > max_dist) {
                r = max_dist;
            }
        }

        // 2. ノイズ除去（移動平均など）
        // ここでは簡易的に実装。必要ならconv_sizeを使って平均化する
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 1. データの前処理
        // -90度〜+90度（またはパラメータ指定）の範囲のみ抽出して処理する
        // インデックス計算
        double fov_deg = this->get_parameter("field_of_view_deg").as_double();
        double fov_rad = fov_deg * M_PI / 180.0;
        
        // 前方が angle 0 と仮定し、-fov/2 ～ +fov/2 の範囲を使う
        // F1TENTHの実機/SIMによっては angle_min/max の定義が異なるが、
        // 一般的にはインデックスの中央が前方0度であることが多い。
        
        int n_ranges = msg->ranges.size();
        std::vector<double> proc_ranges(n_ranges);
        
        // コピーと前処理
        double bubble_radius = this->get_parameter("bubble_radius").as_double();
        double min_dist = 10000.0;
        int closest_idx = -1;

        for (int i = 0; i < n_ranges; ++i) {
            double r = msg->ranges[i];
            
            // 範囲チェック (FOV外は0にして無視する手もあるが、バブル計算には全周囲使ったほうがいいかも)
            // ここではシンプルに全範囲を使用し、後でクリップする
            
            if (std::isnan(r) || std::isinf(r)) {
                r = 0.0;
            }
            
            // 最も近い障害物を探す
            if (r > 0.1 && r < min_dist) { // 0.1m未満はノイズまたは車体自身の可能性
                min_dist = r;
                closest_idx = i;
            }
            
            proc_ranges[i] = r;
        }

        // 2. Safety Bubble の作成
        // 最も近い障害物の周囲を0（壁）にして、そこを通らないようにする
        if (closest_idx != -1) {
            double angle_increment = msg->angle_increment;
            // バブルの半径に相当するインデックス幅を計算
            // arc_length = radius * theta -> theta = radius / distance
            // ここでは単純に ユークリッド距離でバブルを定義
            // しかしLiDAR配列上では角度ベースで消し込むのが簡単
            
            // arctan(bubble_radius / min_dist) でバブルが張る角度を計算
            double bubble_angle = std::atan2(bubble_radius, min_dist);
            int bubble_idx_width = static_cast<int>(bubble_angle / angle_increment);
            
            int start_idx = std::max(0, closest_idx - bubble_idx_width);
            int end_idx = std::min(n_ranges - 1, closest_idx + bubble_idx_width);
            
            for (int i = start_idx; i <= end_idx; ++i) {
                proc_ranges[i] = 0.0;
            }
        }
        
        // 3. Max Gap の探索
        // 前処理として、遠すぎる点を max_lidar_dist にクリップしてからGapを探す
        // Gapとは「閾値以上の距離がある連続領域」
        preprocess_lidar(proc_ranges, msg->range_max); // ここでMaxDistに丸める

        int max_gap_start = -1;
        int max_gap_end = -1;
        double max_gap_len = 0;
        
        int current_gap_start = -1;
        double threshold = 0.5; // この距離以上あれば「隙間」とみなす
        
        for (int i = 0; i < n_ranges; ++i) {
            if (proc_ranges[i] > threshold) {
                if (current_gap_start == -1) {
                    current_gap_start = i;
                }
            } else {
                if (current_gap_start != -1) {
                    // Gap終了
                    int current_gap_len = i - current_gap_start;
                    if (current_gap_len > max_gap_len) {
                        max_gap_len = current_gap_len;
                        max_gap_start = current_gap_start;
                        max_gap_end = i - 1;
                    }
                    current_gap_start = -1;
                }
            }
        }
        // ループ終了時のチェック
        if (current_gap_start != -1) {
             int current_gap_len = n_ranges - current_gap_start;
             if (current_gap_len > max_gap_len) {
                 max_gap_len = current_gap_len;
                 max_gap_start = current_gap_start;
                 max_gap_end = n_ranges - 1;
             }
        }

        // 4. 目標点の決定 (Find Best Point)
        // Max Gapの中で最も遠い点を選ぶ -> クリップされていると「右端（インデックス小）」が選ばれてしまう問題があるため
        // 「Gapの中心」を選ぶように変更する。これが最も直進安定性が高い。
        int best_idx = -1;
        
        // Gapが見つからなかった場合の安全策（直進 or 停止）
        if (max_gap_start == -1) {
            best_idx = n_ranges / 2; // 中央
        } else {
            // Gapの中心を目指す
            best_idx = (max_gap_start + max_gap_end) / 2;
        }

        // ステアリング計算
        double steering_angle = 0.0;
        if (best_idx != -1) {
            double best_angle = msg->angle_min + best_idx * msg->angle_increment;
            steering_angle = best_angle;
        }

        // 速度計算
        double speed = this->get_parameter("speed").as_double();
        // カーブがきつい（ステアリング切ってる）ときは減速
        if (std::abs(steering_angle) > 0.35) { // ~20 deg
            speed = std::max(0.5, speed * 0.5);
        } else if (std::abs(steering_angle) > 0.17) { // ~10 deg
             speed = std::max(1.0, speed * 0.8);
        }

        // Publish
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header = msg->header;
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = speed;
        drive_pub_->publish(drive_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveGapFollower>());
    rclcpp::shutdown();
    return 0;
}