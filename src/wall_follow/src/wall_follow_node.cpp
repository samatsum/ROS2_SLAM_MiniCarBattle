#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <cmath>
#include <vector>
#include <algorithm>

// 定数定義
static constexpr double PI = 3.14159265359;

class WallFollowNode : public rclcpp::Node {
public:
    WallFollowNode() : Node("wall_follow_node") {
        // --- レース用PIDパラメータ調整 ---
        // 高速域ではDゲイン(抑制)が重要になります
        this->declare_parameter("kp", 1.0);//ハンドルの切れ具合
        this->declare_parameter("ki", 0.0);
        this->declare_parameter("kd", 0.05); // 振動抑制
        
        // --- 走行設定 ---
        this->declare_parameter("target_dist", 0.55);    // 壁からの目標距離=0.55m
        this->declare_parameter("lookahead_dist", 0.8); // 予見距離=1.0m
        
        // --- 速度設定 (Dynamic Speed) ---
        this->declare_parameter("max_speed", 4.0);      // 直線番長
        this->declare_parameter("corner_speed", 1.5);   // コーナーは安全に
        this->declare_parameter("slow_steering_threshold", 0.25); // 減速を開始するハンドルの切れ角(rad)

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&WallFollowNode::scan_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);

        RCLCPP_INFO(this->get_logger(), "Race Mode Initialized: Dynamic Speed Enabled.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    double prev_error_ = 0.0;
    double integral_ = 0.0;

    // 角度からインデックスを取得
    int get_index_from_angle(const sensor_msgs::msg::LaserScan::SharedPtr msg, double angle_rad) {
        if (angle_rad < msg->angle_min || angle_rad > msg->angle_max) return -1;
        return static_cast<int>((angle_rad - msg->angle_min) / msg->angle_increment);
    }

    // 距離取得（無効値フィルタ付き）
    double get_range(const sensor_msgs::msg::LaserScan::SharedPtr msg, int index) {
        if (index < 0 || index >= static_cast<int>(msg->ranges.size())) return std::nan("");
        double r = msg->ranges[index];
        if (!std::isfinite(r) || std::isnan(r)) return 10.0; // 無効値は遠くにあるとみなす
        return r;
    }

    // ★速度計算ロジック (Corner Braking)
    double process_speed(double steering_angle) {
        double max_speed = this->get_parameter("max_speed").as_double();
        double corner_speed = this->get_parameter("corner_speed").as_double();
        double threshold = this->get_parameter("slow_steering_threshold").as_double();

        // ハンドルを大きく切っていたら減速
        if (std::abs(steering_angle) > threshold) {
            return corner_speed;
        } 
        
        // 線形補間で滑らかに加速（オプション）
        // 今回はシンプルに、閾値以下なら最高速へ
        return max_speed;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // --- 左壁追従 (Left Wall Follow) ---
        // 高速走行時は視野を少し前に向けたほうが安定します
        double angle_b = PI / 2.0; // 真横 (90度)
        double angle_a = PI / 3.0; // 斜め前 (60度) - 45度より少し前に設定

        int idx_b = get_index_from_angle(msg, angle_b);
        int idx_a = get_index_from_angle(msg, angle_a);

        double range_b = get_range(msg, idx_b);
        double range_a = get_range(msg, idx_a);

        if (std::isnan(range_a) || std::isnan(range_b)) return;

        // 壁との角度 alpha を計算
        double theta = std::abs(angle_a - angle_b);
        double numerator = range_a * std::cos(theta) - range_b;
        double denominator = range_a * std::sin(theta);
        double alpha = std::atan2(numerator, denominator);

        // 現在の壁との距離 D_t
        double dist_t = range_b * std::cos(alpha);

        // 未来の壁との距離 D_t+1 (Lookahead)
        double lookahead_dist = this->get_parameter("lookahead_dist").as_double();
        double dist_future = dist_t + lookahead_dist * std::sin(alpha);

        double target_dist = this->get_parameter("target_dist").as_double();
        
        // エラー計算: 左壁基準
        // 壁に近い(dist_future小) -> エラー正 -> ハンドル右(マイナス)に切りたい
        double error = target_dist - dist_future;

        // PID制御
        double dt = 0.1; // 簡易dt
        double P = this->get_parameter("kp").as_double() * error;
        double I = integral_ + (this->get_parameter("ki").as_double() * error * dt);
        double D = this->get_parameter("kd").as_double() * (error - prev_error_) / dt;

        double steering_angle = P + I + D;
        
        // 左壁なので符号反転 (エラーが正のとき、右に切りたい＝マイナスの操舵角)
        steering_angle = -1.0 * steering_angle;

        // 操舵角リミット (物理限界)
        steering_angle = std::clamp(steering_angle, -0.4189, 0.4189); // 約24度

        // ★ここで速度を動的に決定
        double speed = process_speed(steering_angle);

        integral_ = I;
        prev_error_ = error;

        // 指令値の発行
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header = msg->header;
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = speed;
        
        drive_pub_->publish(drive_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollowNode>());
    rclcpp::shutdown();
    return 0;
}