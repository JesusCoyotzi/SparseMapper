#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <iostream>

//This class listen for a transform and saves to a file
class tfSaver {
private:
  ros::NodeHandle nh_;
  tf::TransformListener listener;
  tf::StampedTransform transform;
  std::string tfFile;
public:
  tfSaver (ros::NodeHandle &nh);
  bool getTransform(std::string frame_orig, std::string frame_target);
  bool saveTransform();
};
