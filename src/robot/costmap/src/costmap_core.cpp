#include "costmap_core.hpp"
#include <algorithm>
#include <cmath>
namespace robot
{

    CostmapCore::CostmapCore(const rclcpp::Logger &logger) : logger_(logger) {}

    void CostmapCore::configure(double resolution, int width,
                                int height, double inflation_radius,
                                int max_cost)
    {
        resolution_ = resolution;
        width_ = width;
        height_ = height;
        inflation_radius_ = inflation_radius;
        max_cost_ = max_cost;

        origin_x_ = -(width_ * resolution_) / 2.0;
        origin_y_ = -(height_ * resolution_) / 2.0;
        grid_.assign(static_cast<size_t>(width_) * height_, 0);
    }

    bool CostmapCore::worldToGrid(double x, double y, int &gx, int &gy) const
    {
        gx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
        gy = static_cast<int>(std::floor((y - origin_y_) / resolution_));

        return (0 <= gx && gx < width_ && 0 <= gy && gy < height_);
    }

    void CostmapCore::markObstacle(int gx, int gy)
    {
        grid_[static_cast<size_t>((gy * width_) + gx)] = static_cast<int8_t>(max_cost_);
    }

    nav_msgs::msg::OccupancyGrid CostmapCore::getCostmap() const
    {
        nav_msgs::msg::OccupancyGrid msg;
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

    void CostmapCore::processScan(const sensor_msgs::msg::LaserScan &scan)
    {
        if (grid_.empty())
        {
            RCLCPP_WARN(logger_, "processScan is called before configure");
            return;
        }

        std::fill(grid_.begin(), grid_.end(), 0);

        for (size_t i = 0; i < scan.ranges.size(); ++i)
        {
            const double r = scan.ranges[i];
            if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max)
            {
                continue;
            }

            const double angle = scan.angle_min + i * scan.angle_increment;
            const double x = r * std::cos(angle);
            const double y = r * std::sin(angle);

            int gx, gy;
            if (worldToGrid(x, y, gx, gy))
            {
                markObstacle(gx, gy);
            }
        }

        inflateObstacles();
    }

    void CostmapCore::inflateObstacles()
    {
        const int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
        const int total = width_ * height_;

        std::vector<int> obstacle_indices;
        for (int i = 0; i < total; ++i)
        {
            if (grid_[i] >= max_cost_)
            {
                obstacle_indices.push_back(i);
            }
        }

        for (int idx : obstacle_indices)
        {
            const int ox = idx % width_;
            const int oy = idx / width_;

            for (int dy = -inflation_cells; dy <= inflation_cells; ++dy)
            {
                for (int dx = -inflation_cells; dx <= inflation_cells; ++dx)
                {
                    const int nx = ox + dx;
                    const int ny = oy + dy;

                    if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_)
                    {
                        continue;
                    }

                    const double dist = std::sqrt(dx * dx + dy * dy) * resolution_;
                    if (dist > inflation_radius_)
                    {
                        continue;
                    }

                    const int cost = static_cast<int>(max_cost_ * (1.0 - dist / inflation_radius_));
                    const int neighbor = ny * width_ + nx;
                    if (cost > grid_[neighbor])
                    {
                        grid_[neighbor] = static_cast<int8_t>(cost);
                    }
                }
            }
        }
    }

};