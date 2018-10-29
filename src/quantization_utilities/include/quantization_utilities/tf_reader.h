#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <fstream>
#include <iostream>

class tfReader {
private:
ros::NodeHandle nh_;
tf2_ros::StaticTransformBroadcaster static_broadcaster;
geometry_msgs::TransformStamped transform;
std::string parent_id, child_id;
public:
tfReader(ros::NodeHandle &nh);
bool loadTf(std::string filepath);
void latchTransform();
};
