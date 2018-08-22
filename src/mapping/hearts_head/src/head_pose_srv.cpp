// C++ standard headers
#include <exception>
#include <string>
#include <math.h>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>

#include <control_msgs/JointTrajectoryControllerState.h>
#include <ros/topic.h>

#include "hearts_nav_msgs/SetHeadPose.h"

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <control_msgs/PointHeadAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define PI 3.14159265

static const std::string headStatusTopic = "/head_controller/state";

// Our Action interface type for moving TIAGo's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
typedef boost::shared_ptr<PointHeadClient> PointHeadClientPtr;

PointHeadClientPtr pointHeadClient;

//Global parameters
double angle_yaw = 0, angle_pitch = 0;
ros::Time latestPoseStamp;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Create a ROS action client to move TIAGo's head
void createPointHeadClient(PointHeadClientPtr& actionClient)
{
  ROS_INFO("Creating action client to head controller ...");

  actionClient.reset( new PointHeadClient("/head_controller/point_head_action") );

  int iterations = 0, max_iterations = 3;
  // Wait for head controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the point_head_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createPointHeadClient: head controller action server not available");
}

void headStatusCallback(const control_msgs::JointTrajectoryControllerState& hstatus)
{  
  trajectory_msgs::JointTrajectoryPoint actualstatus = hstatus.actual;
  latestPoseStamp = hstatus.header.stamp;

  angle_yaw = actualstatus.positions[0];
  angle_pitch = actualstatus.positions[1];
}

bool srvCallback(hearts_nav_msgs::SetHeadPose::Request &req, hearts_nav_msgs::SetHeadPose::Response &res)
{
  //Head parameters
  geometry_msgs::PointStamped pointStamped;
  control_msgs::PointHeadGoal goal;

  //Control head
  pointStamped.header.frame_id = "/xtion_rgb_optical_frame";
  //ss.str(""); ss.clear(); ss << 0;
  pointStamped.header.stamp = latestPoseStamp; //ros::Time(count++); //ss.str();

  goal.pointing_axis.x = 0.0;
  goal.pointing_axis.y = 0.0;
  goal.pointing_axis.z = 1.0;
  goal.pointing_frame = "/xtion_rgb_optical_frame";
  goal.min_duration = ros::Duration(1.0);
  goal.max_velocity = 0.25;

  //Yaw
  double x, y, z;
  x = sin(req.target_yaw*PI/180 + angle_yaw);
  y = 0;
  z = cos(req.target_yaw*PI/180 + angle_yaw);   

  //Pitch + Yaw
  pointStamped.point.x = 0 + x;
  pointStamped.point.y = sin(req.target_pitch*PI/180 + angle_pitch) + y;
  pointStamped.point.z = cos(req.target_pitch*PI/180 + angle_pitch) + z;   

  goal.target = pointStamped;

  pointHeadClient->sendGoal(goal);
  ros::Duration(0.5).sleep();

  res.current_yaw = angle_yaw*180/PI;
  res.current_pitch = angle_pitch*180/PI;
  //ROS_INFO("Sent: %lf, %lf current: %lf, %lf.", req.target_yaw, req.target_pitch, angle_yaw*180/PI, angle_pitch*180/PI);
  
  if ( (fabs(req.target_yaw + angle_yaw*180/PI) > 5) || (fabs(req.target_pitch + angle_pitch*180/PI) > 5) )
	  return false;

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "head_pose_srv");

  ROS_INFO("Starting head_pose_srv application ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh, nh2;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // Create a point head action client to move the TIAGo's head
  createPointHeadClient( pointHeadClient );

  ros::ServiceServer service = nh.advertiseService("/brl_hearts/navigation/head_pose_srv", srvCallback);

  ROS_INFO_STREAM("Subscribing to " << headStatusTopic << " ...");
  ros::Subscriber subhead = nh2.subscribe(headStatusTopic, 1, headStatusCallback);
  
  //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
  ros::Rate loop_rate(5);
  
  ros::spin();

  return EXIT_SUCCESS;
}
