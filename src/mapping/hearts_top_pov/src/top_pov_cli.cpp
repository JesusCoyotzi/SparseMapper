// C++ standard headers
#include <exception>
#include <string>
#include <math.h>

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
#include <hearts_nav_msgs/GetProjectedView.h>

#include <hearts_tools/transformations.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string rgbName         = "RGB Image";
static const std::string povName         = "POV Image";

//Global parameters
bool debug = false;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
        //Server parameters
        hearts_nav_msgs::GetProjectedView srv;
        ///////////

        // Init the ROS node
        ros::init(argc, argv, "top_pov_cli");

        ROS_INFO("Starting top_pov_cli application ...");

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

        ROS_INFO_STREAM("Subscribing to /brl_hearts/navigation/top_pov_srv ...");
        ros::ServiceClient client = nh.serviceClient<hearts_nav_msgs::GetProjectedView>("/brl_hearts/navigation/top_pov_srv");

        ros::Rate loop(10);
        while(ros::ok() && cv::waitKey(15) != 27)
        {
                if (client.call(srv))
                {
                        if ( (srv.response.projected_view.src.width * srv.response.projected_view.src.height) == 0)
                                return 1;

                        cv::Mat src_bgr, src_pc, pov_bgr, pov_pc;
                        Transformations::PointCloud2Msg_ToCvMat(srv.response.projected_view.src, src_bgr, src_pc);
                        Transformations::PointCloud2Msg_ToCvMat(srv.response.projected_view.pov, pov_bgr, pov_pc);

                        if (debug)
                        {
                                cv::imshow(rgbName, src_bgr);
                                cv::imshow(povName, pov_bgr);
                                cv::waitKey(15);
                        }
                } else
                {
                        ROS_ERROR("Failed to call service: top_pov_srv");
                        return 1;
                }

                ros::spinOnce();
                loop.sleep();
        }

        if (debug)
        {
                cv::destroyWindow(rgbName);
                cv::destroyWindow(povName);
        }

        return EXIT_SUCCESS;
}
