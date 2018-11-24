#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"

class poseChck
{
private:
float angularUpdate,linearUpdate;
bool publishCloud, checkPose;
geometry_msgs::PoseStamped previousPose;
ros::Subscriber locSub, slamSub, cloudSub;
ros::Publisher cloudPub;
ros::NodeHandle nh_;
void checkAdvanceCov(const geometry_msgs::PoseWithCovarianceStamped &msg);
void checkAdvance(const geometry_msgs::PoseStamped &msg);

double linearDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
double angularDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

public:
poseChck(ros::NodeHandle &nh);
};
