// C++ standard headers
#include <exception>
#include <string>
#include <math.h>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <ros/topic.h>

#include <nav_msgs/Odometry.h>

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// Hearts headers
#include <hearts_nav_msgs/RobotPose.h>
#include <hearts_nav_msgs/GetRobotPose.h>

#include <hearts_tools/transformations.h>

#define PI 3.14159265
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string odomTopic       = "/hsrb/odom";//= "/mobile_base_controller/odom";
//static const std::string gtodomTopic       = "/ground_truth_odom";

//Odometry trajectory
nav_msgs::Odometry::ConstPtr odometry;

//Global parameters
bool debug = false;
int count = 0;

//uint32_t *vecmap, *veczoom;

std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msgodom)
{
  odometry = msgodom;
}

bool srvCallback(hearts_nav_msgs::GetRobotPose::Request &req, hearts_nav_msgs::GetRobotPose::Response &res)
{
	/*std::cout << "Odometry: " << odometry->pose.pose.position.x << ", " <<
	odometry->pose.pose.position.y << ", " <<
	odometry->pose.pose.position.z << ") (" <<

	odometry->pose.pose.orientation.x << ", " <<
	odometry->pose.pose.orientation.y << ", " <<
	odometry->pose.pose.orientation.z << ", " <<
	odometry->pose.pose.orientation.w << ") " << std::endl;*/

	/*std::cout << "origin: (" <<
	info.origin.position.x << ", " <<
	info.origin.position.y << ", " <<
	info.origin.position.z << ") (" <<

	info.origin.orientation.x << ", " <<
	info.origin.orientation.y << ", " <<
	info.origin.orientation.z << ", " <<
	info.origin.orientation.w << ") " << std::endl;*/

	///Get robot pose in occupancy map coordinates
	//Get robot position
	cv::Mat pos = cv::Mat::zeros(3, 1, CV_32FC1);
	pos.at<float>(0) = (float)odometry->pose.pose.position.y;
	pos.at<float>(1) = (float)odometry->pose.pose.position.x;
	pos.at<float>(2) = 1;

	//Get occupancy map transformation parameters
	cv::Mat k = cv::Mat::zeros(3, 3, CV_32FC1);
	k.at<float>(0,0) = 6.46; k.at<float>(1,1) = 4.84;
	k.at<float>(0,2) = 320; k.at<float>(1,2) = 240;
	k.at<float>(2,2) = 1;

	//Get robot position in occupancy coordinates
	cv::Mat kpos = cv::Mat::zeros(3, 1, CV_32FC1);
	kpos = k*pos;

	//Set robot origin
	cv::Mat o = cv::Mat::zeros(3, 1, CV_32FC1);
	o.at<float>(0) = 320; o.at<float>(1) = 240;

	//Get robot orientation from quaternion to Euclidean form
	double siny = +2.0 * (odometry->pose.pose.orientation.w * odometry->pose.pose.orientation.z);
	double cosy = +1.0 - 2.0 * (odometry->pose.pose.orientation.z * odometry->pose.pose.orientation.z);
	double yaw = atan2(siny, cosy);

	//Set Rt transformation matrix from robot to occupancy map coordinates
	cv::Mat rot = cv::Mat::zeros(3, 3, CV_32FC1);
	rot.at<float>(0,0) = (float)cos(-1*yaw); rot.at<float>(0,1) = (float)-1*sin(-1*yaw);
	rot.at<float>(1,0) = (float)sin(-1*yaw); rot.at<float>(1,1) = (float)cos(-1*yaw);

	//rot.at<float>(0,2) = kpos.at<float>(0)-320;;
	//rot.at<float>(1,2) = kpos.at<float>(1)-240;
	rot.at<float>(0,2) = kpos.at<float>(0)-o.at<float>(0);;
	rot.at<float>(1,2) = kpos.at<float>(1)-o.at<float>(1);
	rot.at<float>(2,2) = 1;

	//std::cout << "(" << rot.at<float>(0,0) << ", " << rot.at<float>(0,1), rot.at<float>(0,2) << " // "
	//		  << rot.at<float>(1,0) << ", " << rot.at<float>(1,1), rot.at<float>(1,2) << ")" << std::endl;

	///Send messages
	std::vector<float_t> veco, vecpos, veck, vecrot;
	int o_w, o_h, pos_w, pos_h, k_w, k_h, rot_w, rot_h;

	Transformations::CvMatf_ToVectMsg(o, &veco, o_w, o_h);
	Transformations::CvMatf_ToVectMsg(pos, &vecpos, pos_w, pos_h);
	Transformations::CvMatf_ToVectMsg(k, &veck, k_w, k_h);
	Transformations::CvMatf_ToVectMsg(rot, &vecrot, rot_w, rot_h);

	res.robot_pose.id = count++;

	res.robot_pose.o = veco;
	res.robot_pose.o_w = o_w;
	res.robot_pose.o_h = o_h;

	res.robot_pose.pos = vecpos;
	res.robot_pose.pos_w = pos_w;
	res.robot_pose.pos_h = pos_h;

	res.robot_pose.yaw = yaw;

	res.robot_pose.k = veck;
	res.robot_pose.k_w = k_w;
	res.robot_pose.k_h = k_h;

	res.robot_pose.rt = vecrot;
	res.robot_pose.rt_w = rot_w;
	res.robot_pose.rt_h = rot_h;

	if (debug)
	{
		///Display current robot pose
		std::cout << "\n o" << o  << "\n pos" << pos << "\n yaw" << yaw << "\n K" << k << "\n Rt" << rot << std::endl;
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "mapping_pose_srv");

  ROS_INFO("Starting mapping_pose_srv application ...");

  // Precondition: Valid clock
  ros::NodeHandle nh, nh2;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  for (int i = 0; i < argc; i++)
  {
	  if ( strcmp( argv[i], "-debug") == 0 )
		  debug = true;
  }

  ros::ServiceServer service = nh.advertiseService("/brl_hearts/navigation/mapping_pose_srv", srvCallback);

  ROS_INFO_STREAM("Subscribing to " << odomTopic << " ...");
  //std::cout << "With dbg" << debug << '\n';
  ros::Subscriber subodom = nh2.subscribe(odomTopic, 1, odomCallback);

  //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
  ros::Rate loop_rate(5);

  ros::spin();

  return EXIT_SUCCESS;
}
