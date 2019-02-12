#include "sparse_map_server/adjacencyMap.h"
#include "sparse_map_server/types.h"
#include "sparse_map_msgs/MakePlan.h"
#include "sparse_map_msgs/SaveMap.h"

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"

class sparseMapEditor{
private:
  ros::NodeHandle nh_;
  ros::Publisher codesPublisher;
  ros::Subscriber removeSub;
  ros::ServiceServer savingSrv, savingPCDSrv, updateSrv;
  std::string mapFrame, opMode ,opSet;
  graphIO codes;
  void modifyCodes(const geometry_msgs::PointStamped &msg);
  void makeCodesMarkerAndPublish(pointArray &codes, std_msgs::ColorRGBA color, int id=0);
  bool saveNodes(sparse_map_msgs::SaveMap::Request &req,
                  sparse_map_msgs::SaveMap::Response &res);
  bool saveNodesAsPCD(sparse_map_msgs::SaveMap::Request &req,
                  sparse_map_msgs::SaveMap::Response &res);
  bool updateParameters(std_srvs::Empty::Request& request,
                        std_srvs::Empty::Response& response );
public:
  sparseMapEditor(ros::NodeHandle &nh);
};
