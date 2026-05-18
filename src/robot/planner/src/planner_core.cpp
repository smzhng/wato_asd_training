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
    pose.header.frame_id = "sim_world";
    pose.pose.position.x = (x - width / 2.0 + 0.5) * res;
    pose.pose.position.y = -(y - height / 2.0 + 0.5) * res;
    pose.pose.orientation.w = 1.0;
    return pose;
}

// Pure bounds check — no longer binary-blocks cells based on cost
bool PlannerCore::isInBounds(int x, int y, const nav_msgs::msg::OccupancyGrid& map) {
    return x >= 0 && y >= 0 && x < (int)map.info.width && y < (int)map.info.height;
}

// Graduated cost function: free space is cheap, inflation zones cost more,
// walls cost a lot — but nothing is ever completely impassable.
// This lets the robot squeeze through tight corridors as a last resort
// instead of giving up entirely like the old binary threshold did.
double PlannerCore::getCost(int x, int y, const nav_msgs::msg::OccupancyGrid& map) {
    int idx = y * map.info.width + x;
    int8_t v = map.data[idx];
    if (v < 0)   return 5.0;                                        // unknown
    if (v == 0)  return 1.0;                                        // free
    if (v < 50)  return 2.0 + (v / 50.0) * 8.0;                   // low inflation (2–10)
    if (v < 90)  return 10.0 + (v - 50) / 40.0 * 40.0;            // high inflation (10–50)
    return 100.0;                                                    // near wall — very costly but passable
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
    path.header.frame_id = "sim_world";

    CellIndex start = worldToGrid(robot_pose.position.x, robot_pose.position.y, map);
    CellIndex goal_cell = worldToGrid(goal.point.x, goal.point.y, map);

    if (!isInBounds(start.x, start.y, map) || !isInBounds(goal_cell.x, goal_cell.y, map)) {
        RCLCPP_WARN(logger_, "Start or goal is out of map bounds!");
        return path;
    }

    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;
    std::unordered_set<CellIndex, CellIndexHash> closed_set;  // prevents reprocessing settled nodes

    g_score[start] = 0.0;
    open_set.push(AStarNode(start, heuristic(start, goal_cell)));

    std::vector<std::pair<int,int>> directions = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

    while (!open_set.empty()) {
        CellIndex current = open_set.top().index;
        open_set.pop();

        // Skip stale queue entries — the node was already settled with a better g-score
        if (closed_set.count(current)) continue;
        closed_set.insert(current);

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
            if (!isInBounds(neighbor.x, neighbor.y, map)) continue;
            if (closed_set.count(neighbor)) continue;  // already settled, skip

            // Graduated cost: diagonal moves cost sqrt(2) * terrain, cardinal cost 1 * terrain
            double move_cost = std::hypot(dir.first, dir.second);
            double terrain_cost = getCost(neighbor.x, neighbor.y, map);
            double tentative_g = g_score[current] + move_cost * terrain_cost;

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
