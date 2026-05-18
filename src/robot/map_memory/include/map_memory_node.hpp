#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
public:
  MapMemoryNode();

private:
  robot::MapMemoryCore map_memory_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid latest_costmap_;
  nav_msgs::msg::OccupancyGrid global_map_;
  double robot_x_{0.0};
  double robot_y_{0.0};
  double last_update_x_{0.0};
  double last_update_y_{0.0};
  bool costmap_received_{false};
  bool should_update_{false};

  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void updateMap();
};

#endif