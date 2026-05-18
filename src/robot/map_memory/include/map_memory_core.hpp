#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <vector>
#include <string>
namespace robot
{

  class MapMemoryCore
  {
  public:
    explicit MapMemoryCore(const rclcpp::Logger &logger);

    void configure(double resolution, int width, int height,
                   const std::string &frame_id);

    void integrate(const nav_msgs::msg::OccupancyGrid &local,
                   double robot_x, double robot_y, double robot_yaw);

    nav_msgs::msg::OccupancyGrid getMap() const;

  private:
    rclcpp::Logger logger_;
    std::vector<int8_t> grid_;
    double resolution_;
    int width_;
    int height_;
    double origin_x_;
    double origin_y_;
    std::string frame_id_;

    bool worldToGrid(double x, double y, int &gx, int &gy) const;
  };
}

#endif