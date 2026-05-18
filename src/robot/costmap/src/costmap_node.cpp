#include <chrono>
#include <memory>
#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  const double resolution = this->declare_parameter<double>("resolution", 0.1);
  const int width = this->declare_parameter<int>("width", 240);
  const int height = this->declare_parameter<int>("height", 240);
  const double inflation_radius = this->declare_parameter<double>("inflation_radius", 0.5);
  const int max_cost = this->declare_parameter<int>("max_cost", 100);

  costmap_.configure(resolution, width, height, inflation_radius, max_cost);

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::scanCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  costmap_.processScan(*msg);
  auto grid = costmap_.getCostmap();
  grid.header.stamp = msg->header.stamp;
  grid.header.frame_id = msg->header.frame_id;
  costmap_pub_->publish(grid);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}