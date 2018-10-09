#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include <iostream>
#include <vector>

#include "space_quantization/quantizedSpace.h"
#include "space_quantization/codebook.h"

#include <visualization_msgs/MarkerArray.h>
#include <string>

union charToFloat
{
        char byteStream[4];
        float assembledFloat;
        int assembledInt;

};

typedef struct segmentedPoint3_
{
        float x,y,z;
        int label;
}labelPoint3;

typedef struct Point3_
{
        float x,y,z;
}Point3;

typedef std::vector<geometry_msgs::Point> pointArray;

//This class takes a quantized_space message
//And separates space into free and occupied space
class spaceSeparator {
private:
ros::NodeHandle nh_;
ros::Subscriber spaceSub, octoSub;
ros::Publisher freeCloudPub, occCloudPub,quantizedSpacePub, markerPub;
ros::Publisher freeCodebookPub, occupiedCodebookPub;
ros::Time stamp,voxelStamp;

float freeThr;
std::string cloudFrame, voxelCloudFrame;
bool pubSegSpace;

void spaceCallback(const space_quantization::quantizedSpace &msg);
void makeVizMsgAndPublish(pointArray &codebook);
void makeVizMarkerAndPublish( pointArray &codebook, bool free);
void separateSpaceAndPublish(labelPoint3* space,
                             pointArray codebook,
                             int nPoints);
float makeFloat(unsigned char * byteArray);
int makeInt(unsigned char * byteArray);
int tolabelPoint3(sensor_msgs::PointCloud2 Cloud,
                  labelPoint3 * points);
int toPoint3(sensor_msgs::PointCloud2 Cloud,
         Point3 * points);
void makeCloudHeader(sensor_msgs::PointCloud2 &cloud, int points);
void voxelSeparatorAndPublish(  Point3 * Ps, int nP, pointArray &codebookFree, pointArray &codebookOcc);
void voxelMapCloudCallback(const sensor_msgs::PointCloud2 &msg);
public:
spaceSeparator (ros::NodeHandle &nh_);
void setVQMapCallbacks();
void setVoxelMapCallbacks();
//void makeColors(int *colors,int nClusters);//no need
};
