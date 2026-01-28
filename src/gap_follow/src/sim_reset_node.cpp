#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <cmath>
#include <thread>

class SimResetNode : public rclcpp::Node {
public:
    SimResetNode() : Node("sim_reset_node") {
        // パラメータ: リスポーン位置 (Launchファイルから受け取る)
        this->declare_parameter("spawn_x", 0.0);
        this->declare_parameter("spawn_y", 0.0);
        this->declare_parameter("spawn_theta", 0.0);
        
        // パラメータ: 衝突判定距離 (壁まで何mでアウトとするか)
        this->declare_parameter("crash_dist", 0.25); 

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&SimResetNode::scan_callback, this, std::placeholders::_1));

        reset_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);

        RCLCPP_INFO(this->get_logger(), "Race Manager: Crash Reset Enabled.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr reset_pub_;
    bool is_resetting_ = false;

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (is_resetting_) return;

        double min_dist = 100.0;
        // 全方位のLiDARデータから最短距離を探す
        for (const auto& range : msg->ranges) {
            if (std::isfinite(range) && !std::isnan(range)) {
                if (range < min_dist) min_dist = range;
            }
        }

        double crash_dist = this->get_parameter("crash_dist").as_double();

        // 衝突判定
        if (min_dist < crash_dist) {
            RCLCPP_WARN(this->get_logger(), "CRASH! (Dist: %.2fm) -> Respawning...", min_dist);
            execute_reset();
        }
    }

    void execute_reset() {
        is_resetting_ = true;

        auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";

        // 設定された位置へ戻す
        double sx = this->get_parameter("spawn_x").as_double();
        double sy = this->get_parameter("spawn_y").as_double();
        double stheta = this->get_parameter("spawn_theta").as_double();

        pose_msg.pose.pose.position.x = sx;
        pose_msg.pose.pose.position.y = sy;
        
        // 向き (Yaw -> Quaternion)
        pose_msg.pose.pose.orientation.z = std::sin(stheta / 2.0);
        pose_msg.pose.pose.orientation.w = std::cos(stheta / 2.0);

        reset_pub_->publish(pose_msg);

        // リセット処理の安定化待機 (2秒間は再検知しない)
        std::thread([this]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            this->is_resetting_ = false;
        }).detach();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimResetNode>());
    rclcpp::shutdown();
    return 0;
}