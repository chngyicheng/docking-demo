#pragma once
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace docking_demo {

class DisturbanceNode : public rclcpp::Node {
public:
    DisturbanceNode();

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    bool shouldCorrupt();
    bool inDropoutWindow();

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

    // Parameters
    bool enable_scan_dropout_;
    bool enable_scan_corruption_;

    // Random generators
    std::mt19937 gen_;
    std::uniform_real_distribution<double> uniform_dist_;

    // Timing for dropout bursts
    rclcpp::Time last_dropout_toggle_;
    bool currently_dropping_;
    double dropout_cycle_duration_;
    double dropout_burst_duration_;
};

}  // namespace docking_demo
