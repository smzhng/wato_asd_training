#include <chrono>
#include <memory>
 
#include <vector>
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap",10);
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharePtr msg) {
    float angle_min = scan->angle_min; //just takes in the lidar outputs
    float angle_max = scan->angle_max;
    float angle_increment = scan->angle_increment;
    float range_min = scan->range_min;
    float range_max = scan->range_max;

    const auto &ranges = scan->ranges; //creates alias for the ranges
    int count = ranges.size();
  
}

void CostmapNode::publishCostmap(int width, int origin, float resolution){
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp= this->now();
    grid.header.frame_id = "map";
    grid.info.resolution = resolution;
    grid.info.width = width;
    grid.info.height = width; //square kinda
    
    
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
