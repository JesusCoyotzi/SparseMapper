#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"

#include "space_quantization/quantizedSpace.h"
#include "space_quantization/codebook.h"

#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>

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

typedef geometry_msgs::Point pointGeom;
typedef std::vector<geometry_msgs::Point> pointArray;
typedef std::vector<std::vector<int> > adjacencyList;

//This receies a quantizedSpace message
//With the occupied space and the centroids
//Ensembles an adjacency graph and makes viz messages.

//Ideally will check if free centroids are connected.
class adjacencyGraph {
private:
struct distanceLabel {
        float dist;
        int label;
};
ros::NodeHandle nh_;
ros::Subscriber quantSub, voxelSub,graphMakeSub;
ros::Publisher markerPub;
ros::Time stamp;
int kNeighboors;
float **adjMat;
float maxDist,freeThr,safetyHeight,safetyRadius;
int edges; //number of elements in graph
std::string cloudFrame,graphFile;
std::vector<geometry_msgs::Point> freeCodebook, occCodebook,codebook;
float distance(geometry_msgs::Point p1,
               geometry_msgs::Point p2);
float norm(geometry_msgs::Point dp);
pointGeom makeGeometryMsg(float x, float y, float z);
bool validateFreeCentroid(pointGeom &freeCentroid,
                          float height, float radius);
bool cilynderCollision(pointGeom pi,pointGeom v, pointGeom q,
                       float height, float radius);
void quantizedCallback(const space_quantization::quantizedSpace &msg);
void codebookCallback(const space_quantization::codebook &msg);
void makeGraph(const std_msgs::Empty &msg);
static bool compareDistance(distanceLabel i, distanceLabel j);
void Knn(pointArray centroids, float ** adjG);
void Knn(pointArray centroids, adjacencyList & adjL);
float ** makeAdjacencyMat(int nEdges);
void printAdjacencyMat(float ** adjM, int n);
void printAdjacencyList(adjacencyList l);
void saveAdjGraph(std::string filename, pointArray centroids, float ** adjG);
void saveAdjGraph(std::string filename, pointArray centroids, adjacencyList &adjL);
void makeVizMsgAndPublish(adjacencyList l);
public:
adjacencyGraph (ros::NodeHandle &nh);
~adjacencyGraph();

};
