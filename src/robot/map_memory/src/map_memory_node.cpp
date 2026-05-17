#include <cmath>
#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() 
: Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {

  // Subscribers
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Timer — updates map at 1Hz
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&MapMemoryNode::updateMap, this));

  // Initialize global map
  global_map_.header.frame_id = "sim_world";
  global_map_.info.resolution = 0.1;
  global_map_.info.width = 200;
  global_map_.info.height = 200;
  global_map_.info.origin.position.x = -10.0;
  global_map_.info.origin.position.y = -10.0;
  global_map_.data.assign(200 * 200, -1); // -1 = unknown

  // Publish initial empty map so planner can start
  map_pub_->publish(global_map_);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_received_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  double dx = robot_x_ - last_update_x_;
  double dy = robot_y_ - last_update_y_;
  double distance = std::sqrt(dx * dx + dy * dy);

  if (distance >= 1.5) {
    should_update_ = true;
  }
}

void MapMemoryNode::updateMap() {
  if (!should_update_ || !costmap_received_) return;

  // Merge latest costmap into global map
  for (int cy = 0; cy < (int)latest_costmap_.info.height; cy++) {
    for (int cx = 0; cx < (int)latest_costmap_.info.width; cx++) {
      int costmap_idx = cy * latest_costmap_.info.width + cx;
      int8_t cell_value = latest_costmap_.data[costmap_idx];

      if (cell_value < 0) continue; // skip unknown cells

      // Convert costmap cell to world coordinates
      double world_x = latest_costmap_.info.origin.position.x + (cx + 0.5) * latest_costmap_.info.resolution;
      double world_y = latest_costmap_.info.origin.position.y + (cy + 0.5) * latest_costmap_.info.resolution;

      // Convert world coordinates to global map indices
      int gx = static_cast<int>((world_x - global_map_.info.origin.position.x) / global_map_.info.resolution);
      int gy = static_cast<int>((world_y - global_map_.info.origin.position.y) / global_map_.info.resolution);

      if (gx < 0 || gx >= (int)global_map_.info.width ||
          gy < 0 || gy >= (int)global_map_.info.height) continue;

      int global_idx = gy * global_map_.info.width + gx;
      global_map_.data[global_idx] = cell_value;
    }
  }

  global_map_.header.stamp = this->get_clock()->now();
  map_pub_->publish(global_map_);

  last_update_x_ = robot_x_;
  last_update_y_ = robot_y_;
  should_update_ = false;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}