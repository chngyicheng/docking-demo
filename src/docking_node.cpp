#include "docking_demo/docking_node.hpp"

namespace docking_demo {

DockingNode::DockingNode() : Node("docking_node") {
    this->declare_parameter("target_x", 1.5);
    this->declare_parameter("target_y", 2.0);
    this->declare_parameter("target_theta", 0.0);
    this->declare_parameter("approach_distance_threshold", 0.8);
    this->declare_parameter("final_approach_distance", 0.2);
    this->declare_parameter("docking_distance_threshold", 0.05);
    this->declare_parameter("docking_angle_threshold", 0.05);

    target_x_ = this->get_parameter("target_x").as_double();
    target_y_ = this->get_parameter("target_y").as_double();
    target_theta_ = this->get_parameter("target_theta").as_double();
    approach_distance_threshold_ = this->get_parameter("approach_distance_threshold").as_double();
    final_approach_distance_ = this->get_parameter("final_approach_distance").as_double();
    docking_distance_threshold_ = this->get_parameter("docking_distance_threshold").as_double();
    docking_angle_threshold_ = this->get_parameter("docking_angle_threshold").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&DockingNode::scanCallback, this, std::placeholders::_1));

    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&DockingNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Docking node initialized");
    RCLCPP_INFO(this->get_logger(), "Target: (%.2f, %.2f, %.2f rad)", target_x_, target_y_, target_theta_);
    RCLCPP_INFO(this->get_logger(), "Approach threshold: %.2f m", approach_distance_threshold_);

}

void DockingNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_ = msg;
}

void DockingNode::sendNavigationGoal() {
    auto goal_msg = geometry_msgs::msg::PoseStamped();
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = this->now();

    // Position
    goal_msg.pose.position.x = target_x_;
    goal_msg.pose.position.y = target_y_;
    goal_msg.pose.position.z = 0.0;

    // Orientation - convert target_theta to quaternion
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, target_theta_);  // Roll, Pitch, Yaw
    goal_msg.pose.orientation.x = q.x();
    goal_msg.pose.orientation.y = q.y();
    goal_msg.pose.orientation.z = q.z();
    goal_msg.pose.orientation.w = q.w();

    goal_pub_->publish(goal_msg);
    RCLCPP_INFO(this->get_logger(),
        "Sent Nav2 goal: position (%.2f, %.2f), orientation %.2f rad (%.1f°)",
        target_x_, target_y_, target_theta_, target_theta_ * 180.0 / M_PI);
}

void DockingNode::controlLoop() {
    auto pose_opt = getRobotPose();
    if (!pose_opt.has_value()) {
        return;
    }

    geometry_msgs::msg::Pose current_pose = pose_opt.value();
    double distance = getDistanceToTarget(current_pose);

    // State machine
    switch (docking_state_) {
        case DockingState::IDLE: {
             if (!goal_sent_) {
                sendNavigationGoal();
                goal_sent_ = true;
                RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 to reach docking vicinity...");
            }
            double current_yaw = getRobotYaw(current_pose);
            double yaw_error = std::abs(normalizeAngle(current_yaw - target_theta_));

            bool close_enough = (distance < approach_distance_threshold_);
            bool oriented_correctly = (yaw_error < approach_orientation_tolerance_);

            if (close_enough && oriented_correctly) {
                RCLCPP_INFO(this->get_logger(),
                    "Ready to dock - dist: %.2fm, orientation error: %.1f° - ENTERING DOCKING",
                    distance, yaw_error * 180.0 / M_PI);
                docking_state_ = DockingState::DOCKING;

            } else if (close_enough && !oriented_correctly) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Close to target (%.2fm) but orientation %.1f° off - keep navigating",
                    distance, yaw_error * 180.0 / M_PI);

            } else {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                    "Waiting: %.2fm from target, %.1f° orientation error",
                    distance, yaw_error * 180.0 / M_PI);
            }
            break;
        }
        case DockingState::DOCKING: {
            auto wall_opt = extractWallGeometry();
            auto cmd = geometry_msgs::msg::Twist();

            if (!wall_opt.has_value()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Wall detection failed. Stopping robot.");
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                cmd_vel_pub_->publish(cmd);
                break;
            }

            auto wall = wall_opt.value();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Wall detected - confidence: %.1f%%, inliers: %d, distance: %.3fm",
                wall.confidence * 100.0, wall.inliers, distance);

            double distance_to_wall = std::abs(wall.c);
            double error_distance = distance_to_wall - d_target_;

            double wall_normal_angle = std::atan2(wall.b, wall.a);
            double heading_option1 = normalizeAngle(wall_normal_angle + M_PI);
            double heading_option2 = normalizeAngle(wall_normal_angle);

            // Choose whichever requires less rotation from 0° (forward in base_link)
            double e_theta = (std::abs(heading_option1) < std::abs(heading_option2))
                             ? heading_option1 : heading_option2;

            if (std::abs(error_distance) < distance_tolerance_ && std::abs(e_theta) < angle_tolerance_) {
                RCLCPP_INFO(this->get_logger(), "DOCKED! Final errors - distance: %.4fm, angle: %.4frad",
                            error_distance, e_theta);
                docking_state_ = DockingState::DOCKED;
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                cmd_vel_pub_->publish(cmd);
                break;
            }
            double linear_vel = 0.0;
            double angular_vel = 0.0;

            // Three-phase control strategy
            if (distance_to_wall > final_approach_distance_) {
                // Phase 1: Normal approach - simultaneous control
                if (std::abs(e_theta) > angle_tolerance_) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Phase 1: Rotating to face wall");
                    angular_vel = k_angular_ * e_theta;
                } else {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Phase 1: Moving to wall");
                    linear_vel = k_linear_ * error_distance;
                    angular_vel = k_angular_ * e_theta * 0.3;  // Reduced speed
                }

            } else {
                // Phase 2 & 3: Final approach - sequential control
                if (std::abs(e_theta) > angle_tolerance_) {
                    // Rotate in place until aligned
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Phase 2: Rotating on the spot");
                    angular_vel = k_angular_ * e_theta;
                } else {
                    // Drive straight forward slowly
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Phase 2: Inching to wall");
                    linear_vel = k_linear_ * error_distance * 0.4;  // Reduced speed for precision
                }
            }

            // Apply velocity limits
            linear_vel = std::clamp(linear_vel, -linear_vel_max_, linear_vel_max_);
            angular_vel = std::clamp(angular_vel, -angular_vel_max_, angular_vel_max_);

            // Safety check: if too close and large error, stop
            if (distance_to_wall < 0.05 && std::abs(e_theta) > 0.3) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Too close with poor alignment - stopping");
            }
            // Publish command
            cmd.linear.x = linear_vel;
            cmd.angular.z = angular_vel;
            cmd_vel_pub_->publish(cmd);

            // Debug logging
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Docking - dist_err: %.3fm, angle_err: %.3frad, v: %.3f, ω: %.3f, confidence: %.1f%%",
                error_distance, e_theta, linear_vel, angular_vel, wall.confidence * 100.0);

            break;
        }

        case DockingState::DOCKED: {
            // Publish zero velocity once, then do nothing
            if (!stopped) {
                auto cmd = geometry_msgs::msg::Twist();
                cmd_vel_pub_->publish(cmd);
                stopped = true;
                RCLCPP_INFO(this->get_logger(), "Docking complete!");
            }
            break;
        }

        case DockingState::FAILED: {
            RCLCPP_INFO(this->get_logger(), "Docking failed");
            break;
        }
    }
}

std::optional<geometry_msgs::msg::Pose> DockingNode::getRobotPose() {
    try {
        auto transform = tf_buffer_->lookupTransform(
            "map", "base_link", tf2::TimePointZero);

        geometry_msgs::msg::Pose pose;
        pose.position.x = transform.transform.translation.x;
        pose.position.y = transform.transform.translation.y;
        pose.position.z = transform.transform.translation.z;
        pose.orientation = transform.transform.rotation;

        return pose;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not get robot pose: %s", ex.what());
        return std::nullopt;
    }
}

double DockingNode::getDistanceToTarget(const geometry_msgs::msg::Pose& pose) {
    double dx = pose.position.x - target_x_;
    double dy = pose.position.y - target_y_;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<Eigen::Vector2d> DockingNode::scanToPoints(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
    std::vector<Eigen::Vector2d> points;

    if (!scan) {
        return points;
    }

    // Filter front-facing points (±60 degrees)
    double front_angle_range = M_PI / 3.0;  // 60 degrees

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double range = scan->ranges[i];

        if (std::isnan(range) || std::isinf(range) ||
            range < scan->range_min || range > scan->range_max) {
            continue;
        }

        double angle = scan->angle_min + i * scan->angle_increment;

        // Keep only front-facing points
        if (std::abs(angle) > front_angle_range) {
            continue;
        }

        // Convert to Cartesian in base_link frame
        double x = range * std::cos(angle);
        double y = range * std::sin(angle);

        points.emplace_back(x, y);
    }

    return points;
}

std::optional<DockingNode::LineModel> DockingNode::extractWallGeometry() {
    std::lock_guard<std::mutex> lock(scan_mutex_);

    if (!latest_scan_) {
        RCLCPP_WARN(this->get_logger(), "No scan data available");
        return std::nullopt;
    }

    auto points = scanToPoints(latest_scan_);

    if (points.size() < 10) {
        RCLCPP_WARN(this->get_logger(), "Not enough valid points: %zu", points.size());
        return std::nullopt;
    }

    // RANSAC line fitting
    LineModel best_model;
    best_model.inliers = 0;
    best_model.confidence = 0.0;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points.size() - 1);

    for (int iter = 0; iter < ransac_iterations_; ++iter) {
        int idx1 = dis(gen);
        int idx2 = dis(gen);

        if (idx1 == idx2) {
            continue;
        }

        const auto& p1 = points[idx1];
        const auto& p2 = points[idx2];

        // Compute line through these two points
        // Line: ax + by + c = 0
        double dx = p2.x() - p1.x();
        double dy = p2.y() - p1.y();

        if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) {
            continue;
        }

        // Normal vector to line: (-dy, dx)
        double a = -dy;
        double b = dx;
        double c = -(a * p1.x() + b * p1.y());

        // Normalize: a^2 + b^2 = 1
        double norm = std::sqrt(a * a + b * b);
        a /= norm;
        b /= norm;
        c /= norm;

        if (c > 0) {
            a = -a;
            b = -b;
            c = -c;
        }

        // Count inliers
        int inlier_count = 0;
        for (const auto& pt : points) {
            double distance = std::abs(a * pt.x() + b * pt.y() + c);
            if (distance < ransac_inlier_threshold_) {
                inlier_count++;
            }
        }

        // Update best model
        if (inlier_count > best_model.inliers) {
            best_model.a = a;
            best_model.b = b;
            best_model.c = c;
            best_model.inliers = inlier_count;
        }
    }

    best_model.confidence = static_cast<double>(best_model.inliers) / points.size();

    if (best_model.confidence < min_inlier_ratio_) {
        RCLCPP_WARN(this->get_logger(), "Low confidence wall detection: %.2f%% inliers",
                    best_model.confidence * 100.0);
        return std::nullopt;
    }

    return best_model;
}

double DockingNode::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double DockingNode::getRobotYaw(const geometry_msgs::msg::Pose& pose) {
    // Convert quaternion to yaw
    double siny_cosp = 2.0 * (pose.orientation.w * pose.orientation.z + 
                              pose.orientation.x * pose.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + 
                                     pose.orientation.z * pose.orientation.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace docking_demo
