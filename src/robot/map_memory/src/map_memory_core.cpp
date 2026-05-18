#include "map_memory_core.hpp"
#include <cmath>
namespace robot
{

  MapMemoryCore::MapMemoryCore(const rclcpp::Logger &logger)
      : logger_(logger) {}

  void MapMemoryCore::configure(double resolution, int width, int height,
                                const std::string &frame_id)
  {
    resolution_ = resolution;
    width_ = width;
    height_ = height;
    frame_id_ = frame_id;
    origin_x_ = -(width_ * resolution_) / 2;
    origin_y_ = -(height_ * resolution_) / 2;
    grid_.assign(static_cast<size_t>(width_ * height_), static_cast<int8_t>(-1));
  }

  bool MapMemoryCore::worldToGrid(double x, double y, int &gx, int &gy) const
  {
    gx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    gy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
    return 0 <= gx && gx < width_ && 0 <= gy && gy < height_;
  }

  nav_msgs::msg::OccupancyGrid MapMemoryCore::getMap() const
  {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.frame_id = frame_id_;
    msg.info.resolution = static_cast<float>(resolution_);
    msg.info.width = static_cast<uint32_t>(width_);
    msg.info.height = static_cast<uint32_t>(height_);
    msg.info.origin.position.x = origin_x_;
    msg.info.origin.position.y = origin_y_;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;
    msg.data = grid_;
    return msg;
  }

  void MapMemoryCore::integrate(const nav_msgs::msg::OccupancyGrid &local,
                                double robot_x, double robot_y, double robot_yaw)
  {
    if (grid_.empty())
    {
      RCLCPP_WARN(logger_, "integrate called before configure()");
      return;
    }

    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);

    const double lx0 = local.info.origin.position.x;
    const double ly0 = local.info.origin.position.y;
    const double lres = local.info.resolution;
    const int lw = static_cast<int>(local.info.width);
    const int lh = static_cast<int>(local.info.height);

    for (int ly = 0; ly < lh; ++ly)
    {
      for (int lx = 0; lx < lw; ++lx)
      {
        const int8_t value = local.data[static_cast<size_t>(ly) * lw + lx];

        if (value < 0)
        {
          continue;
        }

        const double rxf = lx0 + (lx + 0.5) * lres;
        const double ryf = ly0 + (ly + 0.5) * lres;

        const double wx = robot_x + cos_yaw * rxf - sin_yaw * ryf;
        const double wy = robot_y + sin_yaw * rxf + cos_yaw * ryf;

        int gx, gy;
        if (!worldToGrid(wx, wy, gx, gy))
        {
          continue;
        }

        const size_t gidx = static_cast<size_t>(gy) * width_ + gx;
        if (grid_[gidx] < 0 || value > grid_[gidx])
        {
          grid_[gidx] = value;
        }
      }
    }
  }
}