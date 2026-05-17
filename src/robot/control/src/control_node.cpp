#include "control_node.hpp"
#include <memory>
#include <cmath>

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
    
    
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::timerCallback, this));

}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = msg; 
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_odom_ = msg;
}

void ControlNode::timerCallback() {
  if (!current_path_ || !robot_odom_) {
    return;
  }

  if (current_path_->poses.empty()) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }

  // Check if robot has reached the final waypoint
  auto &goal = current_path_->poses.back().pose.position;
  auto &robot_pos = robot_odom_->pose.pose.position;
  if (computeDistance(robot_pos, goal) <= goal_tolerance_) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist()); 
    current_path_->poses.clear();
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    return;
  }


  auto lookahead = findLookaheadPoint();
  if (!lookahead) {
    lookahead = current_path_->poses.back();
  }

  cmd_vel_pub_->publish(computeVelocity(*lookahead));
}

// Walk the path and return the first waypoint far enough ahead
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
  cmd.angular.z = 2.0 * angle_error; // proportional: bigger error = sharper turn
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
