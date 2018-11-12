//Some shorthands for common datatypes
#ifndef TYPES_H
#define TYPES_H

#include "geometry_msgs/Point.h"
#include <map>
typedef std::map<int,geometry_msgs::Point> nodes;
typedef geometry_msgs::Point pointGeom;
typedef std::vector<geometry_msgs::Point> pointArray;
typedef std::vector<std::vector<int> > adjacencyList;

#endif    
