#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl_ros/transforms.h"
//Sparse map messages
#include "sparse_map_msgs/codebook.h"
#include "sparse_map_msgs/Reconfigure.h"
#include "sparse_map_msgs/QuantizeCloud.h"
//STL
#include <iostream>
#include <stack>
//Boost
#include "boost/filesystem.hpp"
//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


typedef std::vector<geometry_msgs::Point> pointArray;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZptr;



class cloudSimulation {
//Small class that generates/reads a pointcloud and
//Publish to a topic to test the sparse map stack
//Also lets save and write simulation results.
private:
static const std::array<std::string, 6> validFileType;
ros::NodeHandle nh_;
ros::Publisher cloudPub;
ros::Subscriber codebookSub;
ros::ServiceClient reconfigureClient, segmentationClient;
ros::Time startTime;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
std::string pcdFile,pcdFolder,csvFile,csvFolder,method, frame;
std::string fullResultsPath;
std::stack<std::string> cloudFiles;
int clusters,iterations;
int maxClusters, minClusters, clustersStep;
int cloudSize;
int simCounter,simTimes,totalSimulations;
float conversionFactor,voxSize;

void sendCloud();
bool makeCloudFromCloudImage(cv::Mat & pcdImg);
bool makeCloudFromDepthImage(cv::Mat & depthImg);
void subsampleCloud();
void codebookCallback(const sparse_map_msgs::codebook &msg);
double L1Norm(pcl::PointXYZ cloudPoint, geometry_msgs::Point cdbkPoint );
double L2Norm(pcl::PointXYZ cloudPoint, geometry_msgs::Point cdbkPoint );
bool writeFileHeader();
bool writeResult(int sim,double secs,double distorsion,
                 double histMean, double histStdDev,
                 unsigned long codesReceived, unsigned long requestedCodes);
bool writeResult(int sim,double secs,double distorsion,
                 unsigned long codesReceived,
                 unsigned long requestedCodes);
double getDistorsion(sparse_map_msgs::codebook cdbk,
                     std::vector<int> partition);
double getDistorsion(pointArray cdbk);
void getHistStats(std::vector<int> &hist,double &mean,double &stdDev);

void printHist( std::vector<int> histogram);

public:
cloudSimulation(ros::NodeHandle &nh);
unsigned int getCloudFiles();
bool loadCloud();
bool loadNextCloud();
void startMonteCarlo();
void monteCarlo();
};
