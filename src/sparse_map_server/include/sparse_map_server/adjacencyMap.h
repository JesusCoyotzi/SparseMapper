#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"

#include "sparse_map_msgs/codebook.h"
#include "sparse_map_server/types.h"

#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <queue>

struct graphNode{
  int vertex;
  double cost;

  graphNode(int vertex, double cost)
  {
    this->vertex = vertex;
    this->cost = cost;
  }
  bool operator<(const graphNode& a) const
  {
     return a.cost < this->cost;
  }
};

class adjacencyMap {
  //This class implements an adjacency graph on nodes.
  //Pretty much the same as the space_quantization package
  //But hopefully more generic.
private:

  pointArray occupiedNodes,freeNodes;
  adjacencyList adjGraph;
  float safetyHeight,safetyRadius,connectionRadius, maxDist;
  int kNeighboors;

  pointGeom parseCodeLine(std::string line);
  pointGeom makePointGeom(float x, float y , float z);
  std_msgs::ColorRGBA makeColor(float r ,float g, float b, float a);
  std::vector<int> parseGraphEntry(std::string line);
  double euclideanDistance(pointGeom a, pointGeom b);
  int getClosestNode(pointGeom p);
  void printPointGeom(pointGeom p);
public:
  adjacencyMap(std::string  filename);
  adjacencyMap();
  pointArray Astar(pointGeom goal,pointGeom start);
  bool loadMap(std::string filename);
  pointArray getFreeNodes();
  pointArray getOccNodes();
  adjacencyList getGraph();

};
