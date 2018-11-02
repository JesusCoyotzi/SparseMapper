#include <vector>
#include "geometry_msgs/Point.h"


typedef struct _point3 {
        //AoS
        float x;
        float y;
        float z;
} point3;

typedef geometry_msgs::Point geomPoint;

class Cylinder {
  //A simple class to represent a cylinder and check collisions with it
  //Also accepts point3 type and ros messages types
private:
  point3 q1,q2;
  float height,radius;

public:
  //Cylinder (point3 q1, point3 q2 , float radius);
  //Cylinder (point3 q1, point3 v, float height=1.0, float radius);
  Cylinder (geometry_msgs::Point q1, geometry_msgs::Point v, float height, float radius);
  bool checkCollsion(geometry_msgs::Point q);

};
