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
//Pretty much the same as the space_quantization package
//But hopefully more generic.
private:

struct graphNode {
        int vertex;
        double cost;

        graphNode(int vertex, double cost);
        bool operator<(const graphNode& a) const;

};

struct distanceLabel {
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
pointArray getFreeNodes();
pointArray getOccNodes();
adjacencyList getGraph();
bool validateTerminals(pointGeom strt,pointGeom goal,
                       int &closetsNodeStrtIdx, int &closestNodeGoalIdx);
bool validateTerminalsQuick(pointGeom strt,pointGeom goal,
                            int &closetsNodeStrtIdx, int &closestNodeGoalIdx);
float getClosestNodeDistance(pointGeom p1);
bool removeOccupiedPoint(pointGeom p);
};

class graphIO
{
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
  bool nodesLoaded;
  bool parseCodeLine(std::string, pointGeom &g);

public:
  double thres;
  graphIO();
  graphIO(std::string filename);
  pointGeom removeOccCode(pointGeom p);
  pointGeom removeFreeCode(pointGeom p);
  bool loadNodes(std::string filename);
  bool saveAsTxt(std::string filename);
  void addFreeCode(pointGeom p);
  void addOccCode(pointGeom p);
  pointArray getFreeCodes();
  pointArray getOccCodes();
  void loadToGraph(adjacencyMap &graph);
};
