#include "sparse_map_server/adjacencyMap.h"
#include "sparse_map_server/types.h"
#include "sparse_map_msgs/MakePlan.h"

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"


class sparseMapServer {
private:
ros::NodeHandle nh_;
ros::Publisher codebookMarkerPub, graphMarkerPub, pathPub;
ros::ServiceServer pathServer;
std::string mapFileName,mapFrame;
float safetyHeight,safetyRadius,connectionRadius, maxDist;
adjacencyMap sparseMap;
int kNeighboors;
void makeCentroidsMarkerAndPublish( pointArray &codebook,std_msgs::ColorRGBA color, int id);
void makeVizGraphAndPublish(adjacencyList l, pointArray codebook);
std_msgs::ColorRGBA makeColor(float r,float g, float b, float a);
bool getPlan(sparse_map_msgs::MakePlan::Request &req,
             sparse_map_msgs::MakePlan::Response &res);
public:
sparseMapServer(ros::NodeHandle &nh);

};
