#include "docking_demo/disturbance_node.hpp"

namespace docking_demo {

DisturbanceNode::DisturbanceNode()
    : Node("disturbance_node"),
      gen_(std::random_device{}()),
      uniform_dist_(0.0, 1.0),
      currently_dropping_(false),
      dropout_cycle_duration_(2.0),   // 2 second cycles
      dropout_burst_duration_(0.5)    // 0.5s dropout bursts
{
    this->declare_parameter("enable_scan_dropout", false);
    this->declare_parameter("enable_scan_corruption", false);

    enable_scan_dropout_ = this->get_parameter("enable_scan_dropout").as_bool();
    enable_scan_corruption_ = this->get_parameter("enable_scan_corruption").as_bool();

    // Subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&DisturbanceNode::scanCallback, this, std::placeholders::_1));

    // Publishers
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_disturbed", 10);

    last_dropout_toggle_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Disturbance node initialized");
    RCLCPP_INFO(this->get_logger(), "  Scan dropout: %s", enable_scan_dropout_ ? "enabled" : "disabled");
    RCLCPP_INFO(this->get_logger(), "  Scan corruption: %s", enable_scan_corruption_ ? "enabled" : "disabled");
}

bool DisturbanceNode::inDropoutWindow() {
    auto elapsed = (this->now() - last_dropout_toggle_).seconds();

    if (currently_dropping_) {
        if (elapsed > dropout_burst_duration_) {
            currently_dropping_ = false;
            last_dropout_toggle_ = this->now();
            RCLCPP_INFO(this->get_logger(), "Dropout burst ended - scans resuming");
        }
        return true;
    } else {
        if (elapsed > dropout_cycle_duration_) {
            currently_dropping_ = true;
            last_dropout_toggle_ = this->now();
            RCLCPP_WARN(this->get_logger(), "Dropout burst started - dropping scans for %.1fs",
                        dropout_burst_duration_);
        }
        return false;
    }
}

bool DisturbanceNode::shouldCorrupt() {
    // Corrupt 20% of scans randomly
    return uniform_dist_(gen_) < 0.2;
}

void DisturbanceNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Handle dropout
    if (enable_scan_dropout_ && inDropoutWindow()) {
        return; // Don't publish
    }

    auto disturbed_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);

    // Handle corruption - corrupt front sector (±30°) which overlaps with docking detection window
    if (enable_scan_corruption_ && shouldCorrupt()) {
        // TurtleBot3 LDS-01: 360 points, angle_min ≈ 0, angle_increment ≈ 0.0175 rad
        // Front is at index 0. Corrupt ±30° = indices 0-17 and 343-359
        int sector_half = static_cast<int>((M_PI / 6.0) / disturbed_scan->angle_increment);
        int total_pts = static_cast<int>(disturbed_scan->ranges.size());

        // Corrupt 0 to +30°
        for (int i = 0; i < sector_half && i < total_pts; ++i) {
            disturbed_scan->ranges[i] = std::numeric_limits<float>::infinity();
        }
        // Corrupt -30° to 0 (wrap around end of array)
        for (int i = total_pts - sector_half; i < total_pts; ++i) {
            disturbed_scan->ranges[i] = std::numeric_limits<float>::infinity();
        }

        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Scan corruption injected - front sector [±30°] set to infinity");
    }

    scan_pub_->publish(*disturbed_scan);
}

}  // namespace docking_demo
