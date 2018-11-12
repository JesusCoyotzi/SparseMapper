#include "sparse_map_server/adjacencyMap.h"
#include "sparse_map_server/types.h"

#include "ros/ros.h"


class sparseMapServer {
private:
ros::NodeHandle nh_;
ros::Publisher codebookMarkerPub, graphMarkerPub;
std::string mapFileName,mapFrame;
float safetyHeight,safetyRadius,connectionRadius, maxDist;
adjacencyMap sparseMap;
int kNeighboors;
void makeCentroidsMarkerAndPublish( pointArray &codebook,std_msgs::ColorRGBA color, int id);
void makeVizGraphAndPublish(adjacencyList l, pointArray codebook);
std_msgs::ColorRGBA makeColor(float r ,float g, float b, float a);
public:
sparseMapServer(ros::NodeHandle &nh);

};
