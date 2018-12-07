#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl_ros/transforms.h"
//tf
#include <iostream>


class cloudGenerator {
  //Small class that generates/reads a pointcloud and
  //Publish to a topic to test the sparse map stack

private:
  ros::NodeHandle nh_;
  ros::Publisher cloudPub;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

public:
  cloudGenerator(ros::NodeHandle &nh);

};
