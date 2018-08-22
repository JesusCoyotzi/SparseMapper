// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
//#include <image_transport/image_transport.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <ros/topic.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// PCL headers
#include <pcl/point_cloud.h>

// Hearts headers
#include <hearts_nav_msgs/ProjectedView.h>

#include "hearts_tools/linefinder.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string rgbName         = "RGB Image";
static const std::string povName         = "POV Image";
static const std::string cloudTopic      = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points"; //"/xtion/depth_registered/points";
static const std::string headStatusTopic = "/hsrb/head_trajectory_controller/state";//"/head_controller/state";

//pcl Point Cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
double angle_yaw = 0, angle_pitch = 0;

//Change of perspective stuff
pointOfViewParameters povParams;

//Global parameters
bool debug = false;

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

void headStatusCallback(const control_msgs::JointTrajectoryControllerState& hstatus)
{  
  trajectory_msgs::JointTrajectoryPoint actualstatus = hstatus.actual;
  
  angle_yaw = actualstatus.positions[0];
  angle_pitch = actualstatus.positions[1];
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{  
  if ( (cloud->width * cloud->height) == 0)
    return;

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud, *pclCloud);

  povParams.color = true; //Color or BW debug image
  povParams.fullData = true; //If false, then use a limited height range

  //Define 3D search boinding box in front of the camera
  povParams.areaLimits.maxX = 1; //Width
  povParams.areaLimits.maxZ = 2; // Depth
  povParams.areaLimits.minY = 1.0; //Min height
  povParams.areaLimits.maxY = 1.5; //Max height

  povParams.angle = (-1)*angle_pitch * 180 / 3.14159265358979323846;//headTilt * 180 / 3.14159265358979323846; //Camera pitch angle in degrees
  //std::cout << "change_of_perspective.->Changing perspective" << std::endl;
  changeViewPerspective ( pclCloud, povParams);
  //std::cout << "change_of_perspective.->Perspective changed" << std::endl;

  if (debug)
  {
	cv::imshow(rgbName, povParams.src);
	cv::imshow(povName, povParams.pov);
	cv::waitKey(15);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "top_pov_node");

  ROS_INFO("Starting top_pov_node application ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh, nh2, nh3;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  for (int i = 0; i < argc; i++)
  {
	  //std::cout << argv[i] << std::endl;
	  if ( strcmp( argv[i], "-debug") == 0 )
		  debug = true;
  }

  ros::Publisher pub = nh.advertise<hearts_nav_msgs::ProjectedView>("/brl_hearts/navigation/top_pov/projected_view",1);
  //image_transport::ImageTransport it(nh);
  //image_transport::Publisher pub = it.advertise("/tiago_brl/top_pov/image", 1);
  
  hearts_nav_msgs::ProjectedView pv;

  // Create the window to show TIAGo's camera images
  if (debug)
  {
	cv::namedWindow(rgbName, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(povName, cv::WINDOW_AUTOSIZE);
  }

  ROS_INFO_STREAM("Subscribing to " << headStatusTopic << " ...");
  ros::Subscriber subhead = nh2.subscribe(headStatusTopic, 1, headStatusCallback);

  ROS_INFO_STREAM("Subscribing to " << cloudTopic << " ...");
  ros::Subscriber subcloud = nh3.subscribe(cloudTopic, 1, cloudCallback);
  
  //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
  ros::Rate loop_rate(5);
  
  int count = 0; 
  while (nh.ok())
  {
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr povCloud;
	  pointOfViewParameters2PointCloud(povParams, povCloud);

	  pv.id = count++;
	  pv.angle = (-1)*angle_pitch * 180 / 3.14159265358979323846;
	  pcl::toROSMsg(*pclCloud, pv.src);
	  pcl::toROSMsg(*povCloud, pv.pov);

	  pub.publish(pv);
	  ros::spinOnce();
	  loop_rate.sleep();
  }
  if (debug)
  {
	cv::destroyWindow(rgbName);
	cv::destroyWindow(povName);
  }

  return EXIT_SUCCESS;
}
