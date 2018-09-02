#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl_ros/transforms.h"
//tf
#include <tf/transform_listener.h>

typedef pcl::PointCloud<pcl::PointXYZRGBA> cloudRGBA;
typedef pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudRGBAPtr;
class cloudPreprocessor {
private:
  ros::NodeHandle nh_;
  ros::Subscriber inCloudSub;
  ros::Publisher processedCloudPub;
  tf::TransformListener tf_listener;
  float cropDistance,voxelSize;
  bool cropEnable, voxelEnable;
  std::string baseFrame;
  void processCallback(const sensor_msgs::PointCloud2ConstPtr& input);
  void filterDistanceZ(double maxDist, cloudRGBAPtr cloud);
  void voxelFilter(double vSize,cloudRGBAPtr cloud);
  bool transformCloud(cloudRGBAPtr cloud);
public:
  cloudPreprocessor (ros::NodeHandle & nh);
};
