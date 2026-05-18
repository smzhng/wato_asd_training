#include "planner_core.hpp"
#include <cmath>

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

CellIndex PlannerCore::worldToGrid(double wx, double wy, const nav_msgs::msg::OccupancyGrid& map) {
    double res = map.info.resolution;
    int width = map.info.width;
    int height = map.info.height;
    int x = static_cast<int>(std::floor(wx / res + width / 2.0));
    int y = static_cast<int>(std::floor(-wy / res + height / 2.0));
    return CellIndex(x, y);
}

geometry_msgs::msg::PoseStamped PlannerCore::gridToWorld(int x, int y, const nav_msgs::msg::OccupancyGrid& map) {
    double res = map.info.resolution;
    int width = map.info.width;
    int height = map.info.height;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = (x - width / 2.0 + 0.5) * res;
    pose.pose.position.y = -(y - height / 2.0 + 0.5) * res;
    pose.pose.orientation.w = 1.0;
    return pose;
}

bool PlannerCore::isValid(int x, int y, const nav_msgs::msg::OccupancyGrid& map) {
    if (x < 0 || y < 0 || x >= (int)map.info.width || y >= (int)map.info.height)
        return false;
    int idx = y * map.info.width + x;
    return map.data[idx] < 50;
}

double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

nav_msgs::msg::Path PlannerCore::planPath(
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Pose& robot_pose,
    const geometry_msgs::msg::PointStamped& goal)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";

    CellIndex start = worldToGrid(robot_pose.position.x, robot_pose.position.y, map);
    CellIndex goal_cell = worldToGrid(goal.point.x, goal.point.y, map);

    if (!isValid(start.x, start.y, map) || !isValid(goal_cell.x, goal_cell.y, map)) {
        RCLCPP_WARN(logger_, "Start or goal is invalid!");
        return path;
    }

    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;

    g_score[start] = 0.0;
    open_set.push(AStarNode(start, heuristic(start, goal_cell)));

    std::vector<std::pair<int,int>> directions = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

    while (!open_set.empty()) {
        CellIndex current = open_set.top().index;
        open_set.pop();

        if (current == goal_cell) {
            CellIndex step = goal_cell;
            while (step != start) {
                path.poses.push_back(gridToWorld(step.x, step.y, map));
                step = came_from[step];
            }
            path.poses.push_back(gridToWorld(start.x, start.y, map));
            std::reverse(path.poses.begin(), path.poses.end());
            return path;
        }

        for (auto& dir : directions) {
            CellIndex neighbor(current.x + dir.first, current.y + dir.second);
            if (!isValid(neighbor.x, neighbor.y, map)) continue;

            double tentative_g = g_score[current] + heuristic(current, neighbor);
            if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
                g_score[neighbor] = tentative_g;
                came_from[neighbor] = current;
                open_set.push(AStarNode(neighbor, tentative_g + heuristic(neighbor, goal_cell)));
            }
        }
    }

    RCLCPP_WARN(logger_, "No path found!");
    return path;
}

}