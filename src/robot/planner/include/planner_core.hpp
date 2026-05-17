#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <vector>
#include <unordered_map>
#include <queue>

namespace robot {

struct CellIndex {
    int x, y;
    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}
    bool operator==(const CellIndex &o) const { return x == o.x && y == o.y; }
    bool operator!=(const CellIndex &o) const { return x != o.x || y != o.y; }
};

struct CellIndexHash {
    std::size_t operator()(const CellIndex &idx) const {
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
};

struct AStarNode {
    CellIndex index;
    double f_score;
    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
    bool operator()(const AStarNode &a, const AStarNode &b) {
        return a.f_score > b.f_score;
    }
};

class PlannerCore {
public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    nav_msgs::msg::Path planPath(
        const nav_msgs::msg::OccupancyGrid& map,
        const geometry_msgs::msg::Pose& robot_pose,
        const geometry_msgs::msg::PointStamped& goal);

private:
    rclcpp::Logger logger_;

    CellIndex worldToGrid(double wx, double wy, const nav_msgs::msg::OccupancyGrid& map);
    geometry_msgs::msg::PoseStamped gridToWorld(int x, int y, const nav_msgs::msg::OccupancyGrid& map);
    bool isValid(int x, int y, const nav_msgs::msg::OccupancyGrid& map);
    double heuristic(const CellIndex& a, const CellIndex& b);
};

}

#endif
