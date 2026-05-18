#include "map_memory_node.hpp"
#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  const double resolution = this->declare_parameter<double>("map_resolution", 0.1);
  const int width = this->declare_parameter<int>("map_width", 240);
  const int height = this->declare_parameter<int>("map_height", 240);
  const std::string frame_id = this->declare_parameter<std::string>("map_frame", "sim_world");
  update_distance_ = this->declare_parameter<double>("update_distance", 0.4);
  const double timer_period_s = this->declare_parameter<double>("update_period_sec", 1.0);

  map_memory_.configure(resolution, width, height, frame_id);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  const auto period = std::chrono::duration<double>(timer_period_s);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&MapMemoryNode::timerCallback, this));

  auto initial_map = map_memory_.getMap();
  initial_map.header.stamp = this->now();
  map_pub_->publish(initial_map);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  have_costmap_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  const auto &q = msg->pose.pose.orientation;
  robot_yaw_ = extractYaw(q.x, q.y, q.z, q.w);

  const double dx = robot_x_ - last_fuse_x_;
  const double dy = robot_y_ - last_fuse_y_;
  if (std::sqrt(dx * dx + dy * dy) >= update_distance_) {
    should_update_ = true;
  }
}

void MapMemoryNode::timerCallback() {
  if (!should_update_ || !have_costmap_) return;

  map_memory_.integrate(latest_costmap_, robot_x_, robot_y_, robot_yaw_);

  auto out = map_memory_.getMap();
  out.header.stamp = this->now();
  map_pub_->publish(out);

  last_fuse_x_ = robot_x_;
  last_fuse_y_ = robot_y_;
  should_update_ = false;
}

double MapMemoryNode::extractYaw(double qx, double qy, double qz, double qw) {
  return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}