// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <ros/topic.h>

#include "hearts_nav_msgs/SetHeadPose.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Entry point
int main(int argc, char** argv)
{
  //Server parameters
  hearts_nav_msgs::SetHeadPose srv;

  // Init the ROS node
  ros::init(argc, argv, "head_pose_cli");

  ROS_INFO("Starting head_pose_cli application ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  double yaw = 0, pitch = 60;
  for (int i = 0; i < argc; i++)
  {
	  if ( strcmp( argv[i], "-yaw") == 0 )
		  yaw = atof(argv[++i]);

	  if ( strcmp( argv[i], "-pitch") == 0 )
		  pitch = atof(argv[++i]);
  }

  ROS_INFO_STREAM("Subscribing to /brl_hearts/navigation/head_pose_srv ...");
  ros::ServiceClient client = nh.serviceClient<hearts_nav_msgs::SetHeadPose>("/brl_hearts/navigation/head_pose_srv");

  srv.request.target_yaw = yaw;
  srv.request.target_pitch = pitch;
  bool increase = true; int step = 15;

  ros::Rate loop(1);
  while(ros::ok())
  {
	  if ((increase)&&(srv.request.target_yaw > 60))
		  increase = false;

	  if ((!increase)&&(srv.request.target_yaw < -60))
		  increase = true;
	  
	  if (increase)
		  srv.request.target_yaw += step; 
	  else
		  srv.request.target_yaw -= step; 
	  
	  client.call(srv);

	  ros::spinOnce();
	  loop.sleep();
  }

  return EXIT_SUCCESS;
}
