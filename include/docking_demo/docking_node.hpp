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
    struct LineModel {
        double a, b, c;  // ax + by + c = 0, normalized
        int inliers;
        double confidence;
    };

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void sendNavigationGoal();
    void controlLoop();
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

    bool stopped = false;
    bool goal_sent_ = false;
    int ransac_iterations_ = 50;
    double target_x_;
    double target_y_;
    double target_theta_;
    double approach_distance_threshold_;
    double docking_distance_threshold_;
    double docking_angle_threshold_;
    double final_approach_distance_;
    double ransac_inlier_threshold_ = 0.05;  // 5cm
    double min_inlier_ratio_ = 0.3;  // 30% of points

    // Control gains
    double k_linear_ = 0.3;
    double k_angular_ = 1.5;
    double d_target_ = 0.05;  // Target distance from wall

    // Velocity limits
    double linear_vel_max_ = 0.15;
    double angular_vel_max_ = 0.5;

    // Dead zone thresholds
    double distance_tolerance_ = 0.07;  // 10cm
    double angle_tolerance_ = 0.1;     // ~6 degrees
    double approach_orientation_tolerance_ = 0.34;
};

}  // namespace docking_demo
