#pragma once
#include <mutex>
#include <optional>
#include <random>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>

namespace docking_demo {

class DockingNode : public rclcpp::Node {
public:
    DockingNode();

private:
    enum class DockingState {
        IDLE,
        DOCKING,
        DOCKED,
        FAILED
    };
    enum class DockingPhase {
        APPROACH,
        FINAL
    };
    struct LineModel {
        double a, b, c;  // ax + by + c = 0, normalized
        int inliers;
        double confidence;
    };

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void sendNavigationGoal();
    void controlLoop();
    void handleIdleState();
    void handleDockingState();
    void handleDockedState();
    void handleFailedState();
    double normalizeAngle(double angle);
    double getRobotYaw(const geometry_msgs::msg::Pose& pose);
    std::vector<Eigen::Vector2d> scanToPoints(const sensor_msgs::msg::LaserScan::SharedPtr& scan);
    std::optional<DockingNode::LineModel> extractWallGeometry();
    std::optional<geometry_msgs::msg::Pose> getRobotPose();
    double getDistanceToTarget(const geometry_msgs::msg::Pose& pose);

    DockingState docking_state_ = DockingState::IDLE;

    std::mutex scan_mutex_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool stopped_ = false;
    bool goal_sent_ = false;
    int ransac_iterations_ = 50;
    double target_x_;
    double target_y_;
    double target_theta_;
    double distance_to_target_;
    double approach_distance_threshold_m_;
    double docking_distance_threshold_m_;
    double docking_angle_threshold_rad_;
    double final_approach_distance_;
    double ransac_inlier_threshold_m_ = 0.05;  // 5cm
    double min_inlier_ratio_ = 0.3;  // 30% of points
    rclcpp::Time last_goal_time_;
    geometry_msgs::msg::Pose current_pose_;
    DockingPhase docking_phase_ = DockingPhase::APPROACH;

    // Control gains
    double k_linear_ = 0.3;
    double k_angular_ = 1.5;
    double d_target_ = 0.1;  // Target distance from wall

    // Velocity limits
    double linear_vel_max_m_per_s_ = 0.15;
    double angular_vel_max_m_per_s_ = 0.5;

    // Dead zone thresholds
    double distance_tolerance_m_ = 0.05;  // 5cm
    double angle_tolerance_rad_ = 0.09;     // ~5 degrees
    double approach_orientation_tolerance_m_ = 0.34;
    int dock_confirm_count_ = 0;
    int dock_confirm_required_ = 5;

    // Failure detection
    rclcpp::Time docking_start_time_;
    int consecutive_wall_failures_ = 0;
    int max_consecutive_failures_ = 25;
    double docking_timeout_s_ = 30.0;

    // Pose noise injection (applied in getRobotPose)
    bool enable_pose_noise_ = false;
    std::mt19937 noise_gen_{std::random_device{}()};
    std::normal_distribution<double> pos_noise_dist_{0.0, 0.1};    // ±10cm
    std::normal_distribution<double> yaw_noise_dist_{0.0, 0.087};  // ±5°
};

}  // namespace docking_demo
