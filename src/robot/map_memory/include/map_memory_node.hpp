#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "map_memory_core.hpp"
#include <vector>
#include <cmath>

class MapMemoryNode : public rclcpp::Node {
public:
  MapMemoryNode();

private:
  robot::MapMemoryCore map_memory_;

  static constexpr double RESOLUTION = 0.1;
  static constexpr int MAP_SIZE = 400; // 40m x 40m

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publisher
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Global map
  std::vector<std::vector<int>> global_map_{MAP_SIZE, std::vector<int>(MAP_SIZE, 0)};

  // Data
  nav_msgs::msg::OccupancyGrid latest_costmap_;
  geometry_msgs::msg::Quaternion orientation_;
  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_yaw_{0.0};
  double last_update_x_{0.0};
  double last_update_y_{0.0};
  bool costmap_received_{false};
  bool should_update_{false};
  bool first_update_{true};  // prevents publishing an empty map before any costmap arrives

  // Methods
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void integrateCostmap();
  void publishMap();
  void updateMap();
};

#endif
