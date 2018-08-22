// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <control_msgs/PointHeadAction.h>
#include <sensor_msgs/image_encodings.h>
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string rgbName         = "RGB Source";
static const std::string depthName       = "Depth Source";
static const std::string cameraFrame     = "/xtion_rgb_optical_frame";
static const std::string imageTopic      = "/hsrb/head_rgbd_sensor/rgb/image_raw";
static const std::string depthTopic      = "/hsrb/head_rgbd_sensor/depth_registered/image_raw";
static const std::string cameraInfoTopic = "/hsrb/head_rgbd_sensor/rgb/camera_info";
static const std::string cloudTopic      = "/hsrb/head_rgbd_sensor/depth_registered/points";

// Intrinsic parameters of the camera
cv::Mat cameraIntrinsics;

// Our Action interface type for moving TIAGo's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
typedef boost::shared_ptr<PointHeadClient> PointHeadClientPtr;

PointHeadClientPtr pointHeadClient;

//pcl Point Cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

ros::Time latestImageStamp;
ros::Time latestDepthStamp;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

// ROS call back for every new image received
void imageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{
  latestImageStamp = imgMsg->header.stamp;

  cv_bridge::CvImagePtr cvImgPtr;

  cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
  cv::imshow(rgbName, cvImgPtr->image);
  cv::waitKey(15);
}

// ROS call back for every new image received
void depthCallback(const sensor_msgs::ImageConstPtr& depthMsg)
{
  latestDepthStamp = depthMsg->header.stamp;
  cv_bridge::CvImagePtr cvImgPtr;

  cvImgPtr = cv_bridge::toCvCopy(depthMsg, sensor_msgs::image_encodings::TYPE_32FC1);
  cv::normalize(cvImgPtr->image, cvImgPtr->image, 1, 0, cv::NORM_MINMAX);
  //ROS_INFO_STREAM("Matrix type: " << type2str(cvImgPtr->image.type()));

  cv::imshow(depthName, cvImgPtr->image);
  cv::waitKey(15);
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  if ( (cloud->width * cloud->height) == 0)
    return;

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud, *pclCloud);
}

// OpenCV callback function for mouse events on a window
void onMouseRGB( int event, int u, int v, int, void* )
{
  if ( event != cv::EVENT_LBUTTONDOWN )
      return;

  ROS_INFO_STREAM("Pixel selected (" << u << ", " << v << ") Making TIAGo look to that direction");

  geometry_msgs::PointStamped pointStamped;

  pointStamped.header.frame_id = cameraFrame;
  pointStamped.header.stamp    = latestImageStamp;

  //compute normalized coordinates of the selected pixel
  double x = ( u  - cameraIntrinsics.at<double>(0,2) )/ cameraIntrinsics.at<double>(0,0);
  double y = ( v  - cameraIntrinsics.at<double>(1,2) )/ cameraIntrinsics.at<double>(1,1);
  double Z = 1.0; //define an arbitrary distance
  pointStamped.point.x = x * Z;
  pointStamped.point.y = y * Z;
  pointStamped.point.z = Z;

  //build the action goal
  control_msgs::PointHeadGoal goal;
  //the goal consists in making the Z axis of the cameraFrame to point towards the pointStamped
  goal.pointing_frame = cameraFrame;
  goal.pointing_axis.x = 0.0;
  goal.pointing_axis.y = 0.0;
  goal.pointing_axis.z = 1.0;
  goal.min_duration = ros::Duration(1.0);
  goal.max_velocity = 0.25;
  goal.target = pointStamped;

  pointHeadClient->sendGoal(goal);
  ros::Duration(0.5).sleep();
}

// OpenCV callback function for mouse events on a window
void onMouseDepth( int event, int u, int v, int, void* )
{
  if ( event != cv::EVENT_LBUTTONDOWN )
      return;

  pcl::PointXYZRGB p = pclCloud->at(u,v);
  ROS_INFO_STREAM("Position at pixel (" << u << ", " << v << ") is (" <<
				   (double)p.x << ", " << (double)p.y << ", " << (double)p.z << ")" );

}

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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "tiago_brl_src");

  ROS_INFO("Starting tiago_brl_src application ...");

  // Precondition: Valid clock
  ros::NodeHandle nh, nh2, nh3;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // Get the camera intrinsic parameters from the appropriate ROS topic
  ROS_INFO("Waiting for camera intrinsics ... ");
  sensor_msgs::CameraInfoConstPtr msg = ros::topic::waitForMessage
      <sensor_msgs::CameraInfo>(cameraInfoTopic, ros::Duration(10.0));
  if(msg.use_count() > 0)
  {
    cameraIntrinsics = cv::Mat::zeros(3,3,CV_64F);
    cameraIntrinsics.at<double>(0, 0) = msg->K[0]; //fx
    cameraIntrinsics.at<double>(1, 1) = msg->K[4]; //fy
    cameraIntrinsics.at<double>(0, 2) = msg->K[2]; //cx
    cameraIntrinsics.at<double>(1, 2) = msg->K[5]; //cy
    cameraIntrinsics.at<double>(2, 2) = 1;
  }

  // Create a point head action client to move the TIAGo's head
  createPointHeadClient( pointHeadClient );

  // Create the window to show TIAGo's camera images
  cv::namedWindow(rgbName, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(depthName, cv::WINDOW_AUTOSIZE);

  // Set mouse handler for the window
  cv::setMouseCallback(rgbName, onMouseRGB);
  cv::setMouseCallback(depthName, onMouseDepth);

  // Define ROS topic from where TIAGo publishes images
  image_transport::ImageTransport it(nh);
  // use compressed image transport to use less network bandwidth
  image_transport::TransportHints transportHint("compressed");

  ROS_INFO_STREAM("Subscribing to " << imageTopic << " ...");
  image_transport::Subscriber subrgb = it.subscribe(imageTopic, 1, imageCallback, transportHint);

  ROS_INFO_STREAM("Subscribing to " << depthTopic << " ...");
  ros::Subscriber subdepth = nh2.subscribe(depthTopic, 1, depthCallback);

  ROS_INFO_STREAM("Subscribing to " << cloudTopic << " ...");
  ros::Subscriber subcloud = nh3.subscribe(cloudTopic, 1, cloudCallback);

  //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
  ros::spin();

  cv::destroyWindow(rgbName);
  cv::destroyWindow(depthName);

  return EXIT_SUCCESS;
}
