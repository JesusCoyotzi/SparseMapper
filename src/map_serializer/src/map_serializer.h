#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

//#include "nav_msgs/Path.h"
#include <string>
#include <iostream>
#include <fstream>

class map_serializer
{

ros::NodeHandle nh_;
ros::Subscriber map_sub;

std::string file;

public:
map_serializer(std::string filename);
void serialize(const nav_msgs::OccupancyGrid &map);
};
