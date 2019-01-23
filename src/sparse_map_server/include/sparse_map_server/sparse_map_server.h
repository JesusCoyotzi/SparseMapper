#include "sparse_map_server/adjacencyMap.h"
#include "sparse_map_server/types.h"
#include "sparse_map_msgs/MakePlan.h"
#include "sparse_map_msgs/SaveMap.h"

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/MarkerArray.h"


class sparseMapServer {
private:
ros::NodeHandle nh_;
ros::Publisher codebookMarkerPub, graphMarkerPub, pathPub, labelPub, terminalPub;
ros::Subscriber rebuildPub;
ros::ServiceServer pathServer;
std::string mapFileName,graphFile,mapFrame;
//TODO this is saved on sparseMap, no need to duplicate outside. REMOVE
float safetyHeight,safetyRadius,connectionRadius, maxDist, minDist,maxDistTerm;
adjacencyMap sparseMap;
bool visNodes,visTerminals,validateTerminals;
int kNeighboors;
void makeCentroidsMarkerAndPublish( pointArray &codebook,std_msgs::ColorRGBA color, int id);
void makeVizGraphAndPublish(adjacencyList l, pointArray codebook);
void makeLabelMsgAndPublish(pointArray &codebook,int id);
void makeTerminalsAndPublish(pointGeom start, pointGeom goal);
std_msgs::ColorRGBA makeColor(float r,float g, float b, float a);
void removePoint(const geometry_msgs::PointStamped &msg);
bool getPlan(sparse_map_msgs::MakePlan::Request &req,
             sparse_map_msgs::MakePlan::Response &res);
void remakeGraph(const std_msgs::Empty &msg);
void saveGraph(const std_msgs::Empty &msg);
public:
sparseMapServer(ros::NodeHandle &nh);

};
