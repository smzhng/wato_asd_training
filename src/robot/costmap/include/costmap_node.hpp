#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "costmap_core.hpp"
#include <vector>
#include <cmath>

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

  private:
    robot::CostmapCore costmap_core_;

    std::vector<std::vector<int>> costmap_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

    const float resolution = 0.1;        //resolution in meters for each cell
    const float inflation_radius = 2.0;  // danger zone around obstacles = 2 meters
    const int occupied = 100;            // cost value for an obstacle cell

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void publishCostmap(int width, int origin, float resolution);
};

#endif
