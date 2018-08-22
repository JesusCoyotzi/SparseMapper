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
#include <hearts_nav_msgs/GetProjectedView.h>

#include "hearts_tools/linefinder.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string rgbName         = "RGB Image";
static const std::string povName         = "POV Image";
static const std::string cloudTopic      = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points"; //"/xtion/depth_registered/points";
static const std::string headStatusTopic = "/hsrb/head_trajectory_controller/state";//"/head_controller/state";

//pcl Point Cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
double angle_yaw = 0, angle_pitch = 0;
//double angle_pan = 0, angle_tilt = 0;

//Change of perspective stuff
pointOfViewParameters povParams;

//Global parameters
bool debug = false;
int count = 0;

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
  
  angle_pitch = actualstatus.positions[0]; //tilt
  angle_yaw = actualstatus.positions[1]; //pan
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{  
  if ( (cloud->width * cloud->height) == 0)
    return;

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud, *pclCloud);
}

bool srvCallback(hearts_nav_msgs::GetProjectedView::Request &req, hearts_nav_msgs::GetProjectedView::Response &res)
{
  povParams.angle = (-1)*angle_pitch * 180 / 3.14159265358979323846;//headTilt * 180 / 3.14159265358979323846; //Camera pitch angle in degrees
  //std::cout << "change_of_perspective.->Changing perspective" << std::endl;
  changeViewPerspective ( pclCloud, povParams);
  //std::cout << "change_of_perspective.->Perspective changed" << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr povCloud;
  pointOfViewParameters2PointCloud(povParams, povCloud);

  res.projected_view.id = count++;
  res.projected_view.angle = (-1)*angle_pitch * 180 / 3.14159265358979323846;
  pcl::toROSMsg(*pclCloud, res.projected_view.src);
  pcl::toROSMsg(*povCloud, res.projected_view.pov);

  if (debug)
  {
	cv::imshow(rgbName, povParams.src);
	cv::imshow(povName, povParams.pov);
	cv::waitKey(15);
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "top_pov_srv");

  ROS_INFO("Starting top_pov_srv application ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh, nh2, nh3;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  char *filename; 
  filename = "path/filename.yaml";

  for (int i = 0; i < argc; i++)
  {
	  if ( strcmp( argv[i], "-debug") == 0 )
		  debug = true;

	  if ( strcmp( argv[i], "-conf") == 0 )
	  {
		  filename = "";
		  filename = strdup( argv[++i] );
		  sprintf ( filename, "%s", filename );
	  }
  }

  cv::FileStorage fs;
  fs.open(filename, cv::FileStorage::READ);

  if (!fs.isOpened())
  {
	  std::cerr << "Failed to open " << filename << std::endl;
	  return 1;
  }

  povParams.color = static_cast<int>(fs["color"]) != 0; //Color or BW debug image
  povParams.fullData = static_cast<int>(fs["fullData"]) != 0; //If false, then use a limited height range

  //Define 3D search boinding box in front of the camera
  fs["maxX"] >> povParams.areaLimits.maxX; //Width
  fs["maxZ"] >> povParams.areaLimits.maxZ; // Depth
  fs["minY"] >> povParams.areaLimits.minY; //Min height
  fs["maxY"] >> povParams.areaLimits.maxY; //Max height

  fs.release();


  // Create the window to show TIAGo's camera images
  if (debug)
  {
	cv::namedWindow(rgbName, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(povName, cv::WINDOW_AUTOSIZE);
  }

  ros::ServiceServer service = nh.advertiseService("/brl_hearts/navigation/top_pov_srv", srvCallback);

  ROS_INFO_STREAM("Subscribing to " << headStatusTopic << " ...");
  ros::Subscriber subhead = nh2.subscribe(headStatusTopic, 1, headStatusCallback);

  ROS_INFO_STREAM("Subscribing to " << cloudTopic << " ...");
  ros::Subscriber subcloud = nh3.subscribe(cloudTopic, 1, cloudCallback);
  
  //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
  ros::Rate loop_rate(5);
  
  ros::spin();

  return EXIT_SUCCESS;
}
