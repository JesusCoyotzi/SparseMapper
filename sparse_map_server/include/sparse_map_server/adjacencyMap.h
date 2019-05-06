#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"

#include "sparse_map_msgs/codebook.h"
#include "sparse_map_server/types.h"

#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <string>
#include <algorithm>
#include <queue>


class adjacencyMap {
//This class implements an adjacency graph on nodes.
//Hopefully somewhat generic.
private:

struct graphNode {
        //Graph node used for priority queu
        int vertex;
        double cost;

        graphNode(int vertex, double cost);
        bool operator<(const graphNode& a) const;

};

struct distanceLabel {
        //Helper struct for knn algorithm
        float dist;
        int label;
        bool operator<(const distanceLabel& a) const;
};

//TODO mini kd tree implementation inside.

pointArray occupiedNodes,freeNodes;
adjacencyList adjGraph;
float safetyHeight,safetyRadius,connectionRadius, maxDist,minDist,maxDistTerm;
int kNeighboors;

bool parseCodeLine(std::string line,pointGeom &p);
pointGeom makePointGeom(float x, float y, float z);
std_msgs::ColorRGBA makeColor(float r,float g, float b, float a);
std::vector<int> parseGraphEntry(std::string line);
double euclideanDistance(pointGeom a, pointGeom b);
double chebyshevDistance(pointGeom a, pointGeom b);
double L1Distance(pointGeom a, pointGeom b);
int getClosestNode(pointGeom p);
int getClosestOccNode(pointGeom p);
bool pruneNode(pointGeom p,pointArray &tmpNodes);
void printPointGeom(pointGeom p);
void Knn(pointArray &centroids, adjacencyList & adjL);
bool validateConnection(pointGeom p1, pointGeom p2, float radius);
bool cylinderCollision(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d q, float radius);
void printAdjacencyList(adjacencyList l);
static bool compareDistance(distanceLabel i, distanceLabel j);
bool validateNode(pointGeom p1);
bool validateNode(pointGeom p1,pointArray &codes);
int reduceNodes(std::list<pointGeom> &nodes, std::vector<pointGeom> &oNodes );

public:
adjacencyMap(std::string mapFile,
             float safeHeight,float safeRadius,
             float cRadius, float mxDist, float minDist, float maxDistTerm,
             int kNeighboors);
adjacencyMap(std::string filename);
adjacencyMap();

bool saveGraph(std::string filename);
void makeGraph();
void setParams(float safeHeight,float safeRadius, float cRadius,
               float mxDist,float minDist,int kNeighboors);
bool Astar(int goal,int start, pointArray &fullPath);
bool loadMap(std::string filename);
bool loadFromPCD(std::string filename);
pointArray getFreeNodes();
pointArray getOccNodes();
adjacencyList getEdges();
bool validateSingleTerminal(pointGeom p, int &nodeIdx, pointArray &codesToCheck);
bool validateTerminals(pointGeom strt,pointGeom goal,
                       int &closetsNodeStrtIdx, int &closestNodeGoalIdx);
bool validateTerminalsQuick(pointGeom strt,pointGeom goal,
                            int &closetsNodeStrtIdx, int &closestNodeGoalIdx);
float getClosestNodeDistance(pointGeom p1);
bool removeOccupiedPoint(pointGeom p);
};

class graphIO
{
//Basic in out for graphs  and nodes
private:
struct lessL1
{
        bool operator()(const pointGeom& p1, const pointGeom &p2)
        {
                double p1norm =  p1.x*p1.x+p1.y*p1.y+p1.z*p1.z;
                double p2norm =  p2.x*p2.x+p2.y*p2.y+p2.z*p2.z;
                return p1norm < p2norm;
        }
};
std::list<pointGeom> freeCodes;
std::list<pointGeom> occCodes;
adjacencyList graph;
bool nodesLoaded;
bool parseCodeLine(std::string, pointGeom &g);
bool parsePCDLine(std::string line, pointGeom &g,int & label);

public:
double thres;
graphIO();
graphIO(std::string filename);
pointGeom removeOccCode(pointGeom p);
pointGeom removeFreeCode(pointGeom p);
bool loadNodes(std::string filename);
bool loadPCD(std::string filename);
bool saveAsTxt(std::string filename);
bool saveAsPCD(std::string filename);
bool saveGraph(std::string filename);
int simpleOccZPassThrough(double max, double min);
int simpleFreeZPassThrough(double max, double min);
int simpleZPassThrough(double max, double min);
void addFreeCode(pointGeom p);
void addOccCode(pointGeom p);
pointArray getFreeCodes();
pointArray getOccCodes();
void getNodes(std::list<pointGeom>& occNodes, std::list<pointGeom>& freeNodes);
void loadGraph(adjacencyMap &graph);
};

class voxelGrid {
//Creates an axis oriented voxel grid for fast acces to centroids
private:
typedef   std::list <pointGeom> voxel;
typedef   std::vector<voxel> voxelArray;
voxelArray voxelGrd;
int cellsX,cellsY,cellsZ;   //Number of cells in each axis
float stepX,stepY,stepZ;   //size of voxel in each direction
bool isSet, isReady;
pointGeom minPoint, maxPoint;
void getAOBB(pointArray &points, pointGeom &minCorner, pointGeom &maxCorner);
public:
voxelGrid(float step);
voxelGrid();
void setStep(float step);
void setStep(float stepx,float stepy,float stepz);
void voxelize(std::vector<pointGeom> points);
void printVoxGrid();
pointArray getPointsInVoxel(pointGeom q, bool eigthN=true);
};
