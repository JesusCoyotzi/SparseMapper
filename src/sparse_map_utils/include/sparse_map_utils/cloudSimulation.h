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
//Sparse map messages
#include "sparse_map_msgs/codebook.h"
#include "sparse_map_msgs/Reconfigure.h"
//STL
#include <iostream>

typedef std::vector<geometry_msgs::Point> pointArray;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZptr;

class cloudSimulation {
  //Small class that generates/reads a pointcloud and
  //Publish to a topic to test the sparse map stack
  //Also lets save and write simulation results.
private:
  ros::NodeHandle nh_;
  ros::Publisher cloudPub;
  ros::Subscriber codebookSub;
  ros::ServiceClient reconfigureClient;
  ros::Time startTime;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  std::string pcdFile,csvFile,method, frame;
  std::string fullResultsPath;
  int simulations,clusters,iterations;
  int maxClusters, clustersStep;
  int simCounter;
  void sendCloud();
  void codebookCallback(const sparse_map_msgs::codebook &msg);
  double L1Norm(pcl::PointXYZ cloudPoint, geometry_msgs::Point cdbkPoint );
  double L2Norm(pcl::PointXYZ cloudPoint, geometry_msgs::Point cdbkPoint );
  bool writeFileHeader();
  bool writeResult(int sim,double secs,double distorsion,unsigned long codes);
  double getDistorsion(pointArray cdbk);

public:
  cloudSimulation(ros::NodeHandle &nh);
  bool loadCloud();
  void startMonteCarlo();
};
