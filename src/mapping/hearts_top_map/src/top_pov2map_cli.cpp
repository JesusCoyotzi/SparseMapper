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

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// Hearts headers
#include <hearts_nav_msgs/OccupancyPOVMap.h>
#include <hearts_nav_msgs/GetOccupancyPOVMap.h>

#include <hearts_tools/transformations.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string mapName         = "MAP Image";
static const std::string zoomName        = "ZOOM Image";

//Global parameters
bool debug = false;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
        //Server parameters
        hearts_nav_msgs::GetOccupancyPOVMap srv;
        ///////////

        // Init the ROS node
        ros::init(argc, argv, "top_pov2map_cli");

        ROS_INFO("Starting top_pov2map_cli application ...");

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
                cv::namedWindow(mapName, cv::WINDOW_AUTOSIZE);
        }

        ROS_INFO_STREAM("Subscribing to /brl_hearts/navigation/top_pov2map_srv ...");
        ros::ServiceClient client = nh.serviceClient<hearts_nav_msgs::GetOccupancyPOVMap>("/brl_hearts/navigation/top_pov2map_srv");

        ros::Rate loop(10);
        while(ros::ok() && cv::waitKey(15) != 27)
        {
                if (client.call(srv))
                {
                        if (srv.response.occupancy_pov_map.id < 0)
                                return 1;

                        if ( (srv.response.occupancy_pov_map.width * srv.response.occupancy_pov_map.height) == 0)
                                return 1;

                        cv::Mat occpovmap;
                        int w = srv.response.occupancy_pov_map.width;
                        int h = srv.response.occupancy_pov_map.height;

                        Transformations::VectMsg_ToCvMatC3(srv.response.occupancy_pov_map.occpovmap, w, h, occpovmap);

                        if (debug)
                        {
                                //Find bounding box
                                int top_x = 1000, top_y = 1000, bottom_x = 0, bottom_y = 0;
                                for(int x = 0; x < occpovmap.cols; x++)
                                        for(int y = 0; y < occpovmap.rows; y++)
                                        {
                                                if((occpovmap.at<cv::Vec3b>(y,x)[0] != 0)||
                                                   (occpovmap.at<cv::Vec3b>(y,x)[1] != 0)||
                                                   (occpovmap.at<cv::Vec3b>(y,x)[2] != 0))
                                                {
                                                        if(x<top_x) {top_x = x; }
                                                        if(y<top_y) {top_y = y; }
                                                        if(x>bottom_x) {bottom_x = x; }
                                                        if(y>bottom_y) {bottom_y = y; }
                                                }
                                        }

                                if(bottom_x-top_x > 0 && bottom_y-top_y > 0)
                                {
                                        cv::Rect roi(top_x, top_y, bottom_x-top_x, bottom_y-top_y);

                                        cv::Mat zoommap = cv::Mat::zeros(480, 640, CV_8UC1);
                                        cv::resize(occpovmap(roi), zoommap, zoommap.size(), 0, 0, cv::INTER_CUBIC);
                                        cv::imshow(zoomName, zoommap);
                                }

                                cv::imshow(mapName, occpovmap);

                                cv::waitKey(15);
                        }
                } else
                {
                        ROS_ERROR("Failed to call service: top_pov2map_srv");
                        return 1;
                }

                ros::spinOnce();
                loop.sleep();
        }

        if (debug)
        {
                cv::destroyWindow(mapName);
        }

        return EXIT_SUCCESS;
}
