#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Empty.h"

#include "sparse_map_msgs/SaveMap.h"
#include "sparse_map_msgs/codebook.h"
//#include "sparse_map_msgs/SaveMap.h"

#include <vector>
#include <string>
#include <fstream>
#include <algorithm>

typedef geometry_msgs::Point pointGeom;
typedef std::vector<pointGeom> pointArray;

// class pointGeom: public geometry_msgs::Point {
//   pointGeom(geometry_msgs::Point p);
//   bool operator<(const pointGeom & p1) const;
//   double normL1();
// };

struct lessL1
{
        inline bool operator()(const pointGeom& p1, const pointGeom &p2)
        {
                float p1norm =  p1.x*p1.x+p1.y*p1.y+p1.z*p1.z;
                float p2norm =  p2.x*p2.x+p2.y*p2.y+p2.z*p2.z;
                return p1norm < p2norm;
        }
};

class sparseMapper {
//Class to accumulate centroids of segmentation
private:
ros::NodeHandle nh_;
ros::Subscriber codebookSub, graphClearSub;
ros::ServiceServer graphMakeSub;
ros::Publisher codebookMarkerPub;
std::vector<geometry_msgs::Point> freeCodebook, occCodebook;
float freeThr;
int edges;

ros::Time stamp;
std::string codebookFrame;

void codebookCallback(const sparse_map_msgs::codebook &msg);
void makeCentroidsMarkerAndPublish(pointArray &codebook, std_msgs::ColorRGBA color,int id);
std_msgs::ColorRGBA makeColor(float r,float g, float b, float a);

bool saveSparseMap(std::string filename);
bool saveMapSrv(sparse_map_msgs::SaveMap::Request &req,
                   sparse_map_msgs::SaveMap::Response &res);
void clearGraph(const std_msgs::Empty &msg);
double normL1(pointGeom p);
public:
sparseMapper (ros::NodeHandle &nh);

};
