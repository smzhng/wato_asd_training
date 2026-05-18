#include "control_node.hpp"
#include <memory>
#include <cmath>
#include <chrono>

ControlNode::ControlNode() : rclcpp::Node("pure_pursuit_controller") {
    // Balanced for 2 m/s on a 100 ms control loop:
    //   distance per tick = 2.0 m/s * 0.1 s = 0.2 m
    //   goal_tolerance is ~2.5x that, so the robot has enough room to stop without overshooting
    //   lookahead is 4x goal_tolerance so it aims past the goal until it's close
    linear_speed_ = 2.0;         // forward speed (m/s)
    goal_tolerance_ = 0.5;       // stop within 50 cm of the goal
    lookahead_distance_ = 2.0;   // aim 2 m ahead along the path

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10,
        [this](const nav_msgs::msg::Path::SharedPtr msg) {
            current_path_ = msg;
        });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            robot_odom_ = msg;
        });

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    using namespace std::chrono_literals;
    control_timer_ = this->create_wall_timer(100ms, [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
    // If no active path, stay silent so teleop commands can pass through.
    // The control node should only touch /cmd_vel while actively following a goal.
    if (!current_path_ || !robot_odom_ || current_path_->poses.empty()) {
        return;
    }

    auto &goal = current_path_->poses.back().pose.position;
    auto &robot_pos = robot_odom_->pose.pose.position;
    if (computeDistance(robot_pos, goal) <= goal_tolerance_) {
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());  // stop ONCE
        current_path_.reset();                                // release the path → next ticks early-return
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        return;
    }

    auto lookahead = findLookaheadPoint();
    if (!lookahead) {
        lookahead = current_path_->poses.back();
    }

    cmd_vel_pub_->publish(computeVelocity(*lookahead));
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
    auto &robot_pos = robot_odom_->pose.pose.position;
    for (const auto &pose : current_path_->poses) {
        if (computeDistance(robot_pos, pose.pose.position) >= lookahead_distance_) {
            return pose;
        }
    }
    return std::nullopt;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
    auto &robot_pos = robot_odom_->pose.pose.position;
    double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

    double dx = target.pose.position.x - robot_pos.x;
    double dy = target.pose.position.y - robot_pos.y;

    double target_angle = std::atan2(dy, dx);
    double angle_error = target_angle - robot_yaw;

    while (angle_error >  M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = linear_speed_;
    cmd.angular.z = 2.0 * angle_error;
    return cmd;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
