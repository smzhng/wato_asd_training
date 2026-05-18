#include <cmath>
#include <chrono>
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
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_received_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  orientation_ = msg->pose.pose.orientation;

  // Quaternion to yaw
  double siny_cosp = 2.0 * (orientation_.w * orientation_.z + orientation_.x * orientation_.y);
  double cosy_cosp = 1.0 - 2.0 * (orientation_.y * orientation_.y + orientation_.z * orientation_.z);
  robot_yaw_ = std::atan2(siny_cosp, cosy_cosp);

  double dx = robot_x_ - last_update_x_;
  double dy = robot_y_ - last_update_y_;
  double distance = std::sqrt(dx * dx + dy * dy);

  if (distance >= 1.0) {
    last_update_x_ = robot_x_;
    last_update_y_ = robot_y_;
    should_update_ = true;
  }
}

void MapMemoryNode::integrateCostmap() {
  const int map_width = latest_costmap_.info.width;
  const int map_height = latest_costmap_.info.height;
  const int local_center_x = map_width / 2;
  const int local_center_y = map_height / 2;
  const int global_center = MAP_SIZE / 2;

  // Convert 1D costmap to 2D
  std::vector<std::vector<int>> cost_map_2D(map_height, std::vector<int>(map_width, 0));
  int idx = 0;
  for (int i = 0; i < map_height; i++) {
    for (int j = 0; j < map_width; j++) {
      cost_map_2D[i][j] = (int)latest_costmap_.data[idx++];
    }
  }

  // Rotate and place onto global map
  double sina = std::sin(-robot_yaw_);
  double cosa = std::cos(-robot_yaw_);

  for (int i = 0; i < map_height; i++) {
    for (int j = 0; j < map_width; j++) {
      if (cost_map_2D[i][j] <= 0) continue;

      double relative_x = local_center_x - j;
      double relative_y = local_center_y - i;

      int map_x = global_center + (int)(-relative_y * cosa - relative_x * sina) + (int)(robot_x_ / RESOLUTION);
      int map_y = global_center + (int)(-relative_y * sina + relative_x * cosa) - (int)(robot_y_ / RESOLUTION);

      if (map_x >= 0 && map_x < MAP_SIZE && map_y >= 0 && map_y < MAP_SIZE &&
          cost_map_2D[i][j] > global_map_[map_y][map_x]) {
        global_map_[map_y][map_x] = cost_map_2D[i][j];
      }
    }
  }
}

void MapMemoryNode::publishMap() {
  nav_msgs::msg::OccupancyGrid map;
  map.header.stamp = this->now();
  map.header.frame_id = "sim_world";
  map.info.resolution = RESOLUTION;
  map.info.width = MAP_SIZE;
  map.info.height = MAP_SIZE;
  // Fixed origin: cell (0,0) of the global grid sits at world (-20, -20).
  // This keeps the map anchored to the world frame regardless of where the robot is,
  // so Foxglove draws it in the correct location every time.
  map.info.origin.position.x = -(MAP_SIZE / 2.0) * RESOLUTION;  // -20 m
  map.info.origin.position.y = -(MAP_SIZE / 2.0) * RESOLUTION;  // -20 m
  map.info.origin.position.z = 0.0;
  // No orientation — the global map is always axis-aligned with the world frame.

  map.data.resize(MAP_SIZE * MAP_SIZE, 0);
  for (int i = 0; i < MAP_SIZE; i++) {
    for (int j = 0; j < MAP_SIZE; j++) {
      map.data[i * MAP_SIZE + j] = (int8_t)global_map_[i][j];
    }
  }

  map_pub_->publish(map);
}

void MapMemoryNode::updateMap() {
  if (!should_update_ || !costmap_received_) return;

  // Skip the very first update — the global map is empty and publishing it would
  // give the planner an all-zeros map before any real obstacles are integrated.
  if (first_update_) {
    first_update_ = false;
    costmap_received_ = false;
    return;
  }

  integrateCostmap();
  publishMap();
  should_update_ = false;
  costmap_received_ = false;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
