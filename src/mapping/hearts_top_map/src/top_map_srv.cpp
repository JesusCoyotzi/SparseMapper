// C++ standard headers
#include <exception>
#include <string>
#include <math.h>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <ros/topic.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// Hearts headers
#include <hearts_nav_msgs/OccupancyMap.h>
#include <hearts_nav_msgs/GetOccupancyMap.h>

#include <hearts_tools/transformations.h>

#define PI 3.14159265
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string mapName         = "MAP Image";
static const std::string mapTopic        = "/map";
//static const std::string gtodomTopic       = "/ground_truth_odom";

//Occupancy Grid
nav_msgs::OccupancyGrid::ConstPtr map;

//Global parameters
bool debug = false;
int count = 0;

//uint32_t *vecmap, *veczoom;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msgmap)
{  
  map = msgmap;
}

bool srvCallback(hearts_nav_msgs::GetOccupancyMap::Request &req, hearts_nav_msgs::GetOccupancyMap::Response &res)
{
  ///Occupancy Grid stuff
  std_msgs::Header header = map->header;
  nav_msgs::MapMetaData info = map->info;
  //ROS_INFO("Got map %d %d", info.width, info.height);

  cv::Mat maptmp = cv::Mat::zeros(info.height, info.width, CV_8UC1);
  for (unsigned int x = 0; x < info.width; x++)
    for (unsigned int y = 0; y < info.height; y++)
    {
	  char value = map->data[x+ info.width * y];
	  if(value == -1)
		maptmp.at<uchar>(x,y) = 0; //maptmp.at<uchar>(y,x) = 0;
	  else if(value == 0)
		maptmp.at<uchar>(x,y) = 255; //maptmp.at<uchar>(y,x) = 255;
	  else
		maptmp.at<uchar>(x,y) = 128; //maptmp.at<uchar>(y,x) = 128;
	}

  cv::Mat occmap = cv::Mat::zeros(480, 640, CV_8UC1);
  cv::resize(maptmp, occmap, occmap.size(), 0, 0, cv::INTER_CUBIC);

  ///Send messages
  //delete[] vecmap; delete[] veczoom;
  std::vector<uint32_t> vecmap;//, veczoom;
  int width = 0, height = 0, depth = 1;

  Transformations::CvMat_ToVectMsg(occmap, &vecmap, width, height);
  //Transformations::CvMat_ToVectMsg(zoommap, &veczoom, width, height);

  res.occupancy_map.id = count++;
  res.occupancy_map.width = width;
  res.occupancy_map.height = height;
  res.occupancy_map.depth = depth;
  res.occupancy_map.occmap = vecmap;
  //res.occupancy_map.zoommap = veczoom;

  ///Debug
  if (debug)
  {
	cv::imshow(mapName, occmap);
	//cv::imshow(zoomName, zoommap);
	cv::waitKey(15);
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "top_map_srv");

  ROS_INFO("Starting top_map_srv application ...");
 
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

  // Create the window to show TIAGo's camera images
  if (debug)
  {
	cv::namedWindow(mapName, cv::WINDOW_AUTOSIZE);
  }

  ros::ServiceServer service = nh.advertiseService("/brl_hearts/navigation/top_map_srv", srvCallback);

  ROS_INFO_STREAM("Subscribing to " << mapTopic << " ...");
  ros::Subscriber submap = nh2.subscribe(mapTopic, 1, mapCallback);

  //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
  ros::Rate loop_rate(5);
  
  ros::spin();

  return EXIT_SUCCESS;
}
