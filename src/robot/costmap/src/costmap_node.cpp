#include <chrono>
#include <memory>
 
#include <vector>
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_core_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap",10);
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    float angle_min = scan->angle_min; //just takes in the lidar outputs
    //technically don tneed angle max if i use increments?
    float angle_increment = scan->angle_increment;
    float range_min = scan->range_min; //apparently comes with it in ros
    float range_max = scan->range_max;

    const auto &ranges = scan->ranges; //creates alias for the ranges
    int count = ranges.size(); //basically how many scans it takes

    // required grid size to capture all the lidar signals
    int range_size = static_cast<int>(std::ceil((2.0*range_max)/resolution))+1; //static cast converts it to int basically
    int origin = range_size/2; //tells the robot its in the very center in reference to the lidar scanners

    //makes the grid, in this case, there are range_size # of rows and columns, all filled with 0 for now. does it at the start of every call to start fresh (NECESSARY)
    costmap_.assign(range_size, std::vector<int>(range_size,0));

    for (int i = 0; i<count; i++){ //goes through the full range of beams
        float angle = angle_min + (i*angle_increment); 
        float range = ranges[i];//distance beam travels before hitting something

        if (range < range_min || range>range_max){
            continue; //basically skipping the reading if it's invalid
        }
        //distance*angle to find the x,y coordinates, then adds to the origin to show the point assuming origin is where car is at for its own scan
        int x = static_cast<int>((range*std::sin(angle))/resolution)+origin;
        int y = static_cast<int>((range*std::cos(angle))/resolution)+origin;

        if (x<0 || x>=range_size||y<0||y>=range_size){
            continue; //like the if statement above, skipping if the x and y are out of bounds
        }

        costmap_[y][x] = occupied; //assigns the absolute occupied value to show an object is there (100)

        // inflation (friend harper helped) is spreading the cost to nearby walls so robot avoids them too for safety
        int inflation_cells = static_cast<int>(inflation_radius/resolution);
        //gets all the cells around the occupied cell
        for (int dx = -inflation_cells; dx <= inflation_cells; ++dx){
            for (int dy = -inflation_cells; dy <= inflation_cells; ++dy){
                int nx = x +dx;
                int ny = y + dy;

                float dist = std::sqrt(dx*dx+dy*dy)*resolution; //pythagorean to find distance
                if (nx>=0 && nx<range_size && ny >=0 && ny<range_size && dist<=inflation_radius){
                    int inflated_cost = static_cast<int>(occupied*(1.0-dist/inflation_radius));

                    costmap_[ny][nx] = std::max(costmap_[ny][nx], inflated_cost); //assigns the inflated costs to the tiles
                }
            }
        }

    }
    publishCostmap(range_size, origin, resolution); //finished building grid, now publish and sending it

}

void CostmapNode::publishCostmap(int width, int origin, float resolution){
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp= this->now(); //when this was made and what coordinate frame
    grid.header.frame_id = "map";
    grid.info.resolution = resolution;
    grid.info.width = width;
    grid.info.height = width; //square grid
    
    //since robot is in center according to grid cells, but technically position 0 irl. this chnages for irl to meters as well
    grid.info.origin.position.x = -static_cast<double>(origin) * resolution;
    grid.info.origin.position.y = -static_cast<double>(origin) * resolution;
    grid.info.origin.position.z = 0.0;

    //flattens out the grid from 2d to 1d for ROS
    grid.data.resize(width * width, 0);
    for (int y = 0; y < width; ++y){
        for (int x = 0; x < width; ++x){
            grid.data[y * width + x] = static_cast<int8_t>(costmap_[y][x]);
        }
    }
    costmap_pub_->publish(grid);
    
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
