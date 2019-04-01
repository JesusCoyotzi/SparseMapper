#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include "quantization.h"

#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"

// #include "space_quantization/quantizedSpace.h"
// #include "space_quantization/codebook.h"
#include "sparse_map_msgs/codebook.h"
#include "sparse_map_msgs/Reconfigure.h"
#include "sparse_map_msgs/QuantizeCloud.h"

#include <iostream>
#include <vector>
#include <string>

union charToFloat
{
        char byteStream[4];
        float assembledFloat;
        int assembledInt;

};



/*This class implements the kmeans cuda-based segmentation algorithm
   on a PointCloud2 ros message*/
class spaceSegmenter {
private:
ros::NodeHandle nh_;
ros::Subscriber sub_cloud;
ros::Publisher labeledCloudPub, quantizedSpace_pub, codebook_pub;
ros::Publisher freeCloud, occCloud;
ros::ServiceServer reconfigureService, quantizeService;
bool pubSegSpace;
float freeThr;
int nClusters, iterations;   //number of clusters and iterations to run

std::string cloudFrame, method;
ros::Time stamp;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
bool reconfigureCallback(sparse_map_msgs::Reconfigure::Request & req,
                         sparse_map_msgs::Reconfigure::Response &res);
bool segmenterServer(sparse_map_msgs::QuantizeCloud::Request &req,
                     sparse_map_msgs::QuantizeCloud::Response &res);
bool validateCloudMsg(sensor_msgs::PointCloud2 msg);
float makeFloat(unsigned char * byteArray);
int toPoint3(sensor_msgs::PointCloud2 tfCloud,
             point3 * points);
void getMinMax(point3 * points,
               point3 &max, point3 &min,
               int nPoints);
void getAOBB(point3 * points,
             point3 &max, point3 &min,
             int nPoints);
float norm(point3 p);
void labelSpaceAndPublish(point3* space, point3* codebook,
                          int * partition, int *histogram,
                          int nPoints);
void makePartitionMsg(std::vector<int> &partMsg, int* partition, unsigned int nPoints );
void makeCodebookMsg(std::vector<geometry_msgs::Point> &msg,
                     point3 *codebook, int* histogram, int nClusters);
void makeHistogramMsg(std::vector<int> &histMsg, int* histogram, unsigned int nClusters );
void makeCloudHeader(sensor_msgs::PointCloud2 &cloud, int points);

public:
spaceSegmenter (ros::NodeHandle nh);

~spaceSegmenter ();
};
