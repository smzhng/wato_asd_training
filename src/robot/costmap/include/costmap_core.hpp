#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

  class CostmapCore
  {
  public:
    explicit CostmapCore(const rclcpp::Logger &logger);

    void configure(double resolution, int width, int height,
                   double inflation_radius, int max_cost);

    void processScan(const sensor_msgs::msg::LaserScan &scan);

    nav_msgs::msg::OccupancyGrid getCostmap() const;

  private:
    rclcpp::Logger logger_;
    std::vector<int8_t> grid_;
    double resolution_;
    int width_;
    int height_;
    double origin_x_;
    double origin_y_;
    double inflation_radius_;
    int max_cost_;

    bool worldToGrid(double x, double y, int &gx, int &gy) const;
    void markObstacle(int gx, int gy);
    void inflateObstacles();
  };

}

#endif