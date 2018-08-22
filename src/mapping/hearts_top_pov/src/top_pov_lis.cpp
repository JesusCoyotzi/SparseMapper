// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
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

#include <hearts_tools/transformations.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string rgbName         = "RGB Image";
static const std::string povName         = "POV Image";
static const std::string povTopic        = "/brl_hearts/navigation/top_pov/projected_view";

//Global parameters
bool debug = false;

void povCallback( hearts_nav_msgs::ProjectedView pv )
{
  if ( (pv.src.width * pv.src.height) == 0)
    return;

  cv::Mat src_bgr, src_pc, pov_bgr, pov_pc;
  Transformations::PointCloud2Msg_ToCvMat(pv.src, src_bgr, src_pc);
  Transformations::PointCloud2Msg_ToCvMat(pv.pov, pov_bgr, pov_pc);

  if (debug)
  {
	cv::imshow(rgbName, src_bgr);
	cv::imshow(povName, pov_bgr);
	cv::waitKey(15);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "top_pov_lis");

  ROS_INFO("Starting top_pov_lis application ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh;
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

  // Create the window to show TIAGo's camera images
  if (debug)
  {
	cv::namedWindow(rgbName, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(povName, cv::WINDOW_AUTOSIZE);
  }

  ROS_INFO_STREAM("Subscribing to " << povTopic << " ...");
  ros::Subscriber subpov = nh.subscribe(povTopic, 1, povCallback);

  //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
  ros::spin();

  if (debug)
  {
	cv::destroyWindow(rgbName);
	cv::destroyWindow(povName);
  }

  return EXIT_SUCCESS;
}
