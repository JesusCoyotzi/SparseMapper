#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include <tf/transform_listener.h>
#include <iostream>
#include <vector>
#include "quantization.h"

#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"

#include <visualization_msgs/Marker.h>
#include <string>

union charToFloat
{
        char byteStream[4];
        float assembledFloat;
        int assembledInt;

};

class spaceSegmenter {
private:
ros::NodeHandle nh_;
ros::Subscriber sub_cloud;
ros::Publisher marker_pub, segClouds_pub, freeCloud_pub, occCloud_pub;
tf::TransformListener tf_listener;
float freeThr;
int nClusters, iterations;   //number of clusters and iterations to run
int *colors=NULL;
bool vizCnt, vizSegCloud;
std::string cloudFrame, baseFrame;
visualization_msgs::Marker free_points, occ_points;
bool cloudToBaseLink(
        const sensor_msgs::PointCloud2ConstPtr& in_cloud,
        sensor_msgs::PointCloud2 &out_cloud);
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
float makeFloat(unsigned char * byteArray);
int toPoint3(sensor_msgs::PointCloud2 tfCloud,
             point3 * points);
void getMinMax(point3 * points,
               point3 &max, point3 &min,
               int nPoints);
float norm(point3 p);
void makeVizMsgAndPublish(point3 *codebook, int nClusters);
void makeSegmentedCloudAndPublish(point3 *space, int *partition,
                                  int nPoints);
void makeColors(int *colors,int nClusters);
void separateSpaceAndPublish(point3* space, point3* codebook,
                        int * partition,
                        int nPoints);
public:
spaceSegmenter (ros::NodeHandle nh);

~spaceSegmenter ();
};
