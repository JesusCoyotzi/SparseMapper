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

#include <hearts_nav_msgs/OccupancyPOVMap.h>
#include <hearts_nav_msgs/GetOccupancyPOVMap.h>

#include <hearts_nav_msgs/ProjectedView.h>
#include <hearts_nav_msgs/GetProjectedView.h>

#include <hearts_nav_msgs/RobotPose.h>
#include <hearts_nav_msgs/GetRobotPose.h>

#include <hearts_tools/transformations.h>

#define PI 3.14159265
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string robotName       = "ROBOT map";
static const std::string povName         = "POV map";
static const std::string odomTopic       = "/hsrb/odom"; //"/mobile_base_controller/odom";
//static const std::string gtodomTopic       = "/ground_truth_odom";

//Occupancy Grid
nav_msgs::OccupancyGrid::ConstPtr map;

//Odometry trajectory
nav_msgs::Odometry::ConstPtr odometry;

//Global parameters
bool debug = false;
int count = 0;

//Server parameters
ros::ServiceClient client_pov, client_map, client_pose;
hearts_nav_msgs::GetProjectedView srv_pov;
hearts_nav_msgs::GetOccupancyMap srv_map;
hearts_nav_msgs::GetRobotPose srv_pose;

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

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msgmap)
{
        map = msgmap;
}

bool srvCallback(hearts_nav_msgs::GetOccupancyPOVMap::Request &req, hearts_nav_msgs::GetOccupancyPOVMap::Response &res)
{
        // if(!client_pov.call(srv_pov) || !client_map.call(srv_map) || !client_pose.call(srv_pose))
        // {
        //     std::cout << "something awas not called correctly" << '\n';
        //     cout
        //         return false;
        // }
        if(!client_pov.call(srv_pov))
        {
                std::cout << ";-)Pov service call failed" << '\n';
                return false;

        }
        if(!client_map.call(srv_map))
        {
                std::cout << "°!° Map service call failed" << '\n';
                return false;

        }
        if(!client_pose.call(srv_pose))
        {
                std::cout << "XD Pose service call failed" << '\n';
                return false;

        }

        ///Get colored occupancy grid map
        //Get occupancy grid map
        if (srv_map.response.occupancy_map.id < 0)
        {
                std::cout << "Invalid map received" << '\n';
                return false;
        }

        if ( (srv_map.response.occupancy_map.width * srv_map.response.occupancy_map.height) == 0)
        {
                std::cout << "Another empty map received" << '\n';
                return false;
        }

        cv::Mat occmap;
        int width = srv_map.response.occupancy_map.width;
        int height = srv_map.response.occupancy_map.height;

        Transformations::VectMsg_ToCvMat(srv_map.response.occupancy_map.occmap, width, height, occmap);

        //Color map
        cv::Mat colormap = cv::Mat::zeros(height, width, CV_8UC3);
        cv::Mat pov = cv::Mat::zeros(height, width, CV_8UC3);

        for(int x = 0; x < occmap.cols; x++)
                for(int y = 0; y < occmap.rows; y++)
                {
                        if (occmap.at<uchar>(y,x) != 0)
                        {
                                colormap.at<cv::Vec3b>(y,x)[0] = occmap.at<uchar>(y,x);
                                colormap.at<cv::Vec3b>(y,x)[1] = occmap.at<uchar>(y,x);
                                colormap.at<cv::Vec3b>(y,x)[2] = occmap.at<uchar>(y,x);
                        }
                }

        ///Get robot pose in occupancy map coordinates
        cv::Mat o, pos, k, rt;
        double yaw;

        width = srv_pose.response.robot_pose.o_w;
        height = srv_pose.response.robot_pose.o_h;
        Transformations::VectMsg_ToCvMatf(srv_pose.response.robot_pose.o, width, height, o);

        width = srv_pose.response.robot_pose.pos_w;
        height = srv_pose.response.robot_pose.pos_h;
        Transformations::VectMsg_ToCvMatf(srv_pose.response.robot_pose.pos, width, height, pos);

        yaw = srv_pose.response.robot_pose.yaw;

        width = srv_pose.response.robot_pose.k_w;
        height = srv_pose.response.robot_pose.k_h;
        Transformations::VectMsg_ToCvMatf(srv_pose.response.robot_pose.k, width, height, k);

        width = srv_pose.response.robot_pose.rt_w;
        height = srv_pose.response.robot_pose.rt_h;
        Transformations::VectMsg_ToCvMatf(srv_pose.response.robot_pose.rt, width, height, rt);

        ///Get POV data
        if ( (srv_pov.response.projected_view.src.width * srv_pov.response.projected_view.src.height) == 0)
        {
                std::cout << "POV data empty" << '\n';
                return false;
        }

        pcl::PointCloud<pcl::PointXYZRGBA> pc_pcl;
        pcl::fromROSMsg(srv_pov.response.projected_view.pov, pc_pcl);

        ///Map POV data into occupancy grid map
        cv::Mat p = cv::Mat::zeros(3, 1, CV_32FC1);
        cv::Mat r = cv::Mat::zeros(3, 1, CV_32FC1);
        cv::Mat r_ = cv::Mat::zeros(3, 1, CV_32FC1);

        for (int h=0; h<colormap.rows; h++)
                for (int w=0; w<colormap.cols; w++)
                {
                        pcl::PointXYZRGBA pt = pc_pcl.at(w, h);
                        if((pt.b > 0)&&(pt.g > 0)&&(pt.r > 0))
                        {
                                p.at<float>(0) = -1*pt.x;
                                p.at<float>(1) = pt.z;
                                p.at<float>(2) = 1;

                                /*cv::Mat rt = cv::Mat::zeros(3, 3, CV_32FC1);
                                   rt.at<float>(0,0) = 1; rt.at<float>(0,1) = 0;
                                   rt.at<float>(1,0) = 0; rt.at<float>(1,1) = 1;

                                   rt.at<float>(0,2) = res.at<float>(0)-320;
                                   rt.at<float>(1,2) = res.at<float>(1)-240;
                                   rt.at<float>(2,2) = 1;
                                   r_ = rt*(k*p);*/

                                r = rt*(k*p - o) + o;

                                //std::cout << "p: " << p << ", r_: " << r_ << ", r: " << r << std::endl;

                                int px = int(r.at<float>(0)); int py = int(r.at<float>(1));
                                //std::cout << "(" << pt.x << ", " << pt.y << ", " << pt.z
                                //		  << ") => (" << px << ", " << py << ")" << std::endl;
                                colormap.at<cv::Vec3b>(py,px)[0] = (unsigned char)pt.b;
                                colormap.at<cv::Vec3b>(py,px)[1] = (unsigned char)pt.g;
                                colormap.at<cv::Vec3b>(py,px)[2] = (unsigned char)pt.r;

                                pov.data[h*pov.step + w*3] = (unsigned char)pt.b;
                                pov.data[h*pov.step + w*3 + 1] = (unsigned char)pt.g;
                                pov.data[h*pov.step + w*3 + 2] = (unsigned char)pt.r;
                        }
                }

        //std::cout << "rot: " << rot << ", yaw: " << yaw << std::endl;

        //std::cout << "(" << pos.at<float>(0) << ", " << pos.at<float>(1) << ", " << 180*yaw/PI  << ") => ("
        //		  << res.at<float>(0) << ", " << res.at<float>(1) << ")" << std::endl;

        //std::cout << "(" << res.at<float>(0) << ", " << res.at<float>(1) << ") ("
        //		  << res.at<float>(0)-320 << ", " << res.at<float>(1)-240 << ")" << std::endl;

        ///Send messages
        std::vector<uint32_t> vecmap;
        int depth = 3;

        Transformations::CvMatC3_ToVectMsg(colormap, &vecmap, width, height);

        res.occupancy_pov_map.id = count++;
        res.occupancy_pov_map.width = width;
        res.occupancy_pov_map.height = height;
        res.occupancy_pov_map.depth = depth;
        res.occupancy_pov_map.occpovmap = vecmap;

        if (debug)
        {
                ///Plot current robot pose
                //Get robot position in occupancy coordinates
                cv::Mat kpos = cv::Mat::zeros(3, 1, CV_32FC1);
                kpos = k*pos;

                cv::Point center(kpos.at<float>(0), kpos.at<float>(1));
                int thickness = -1; int lineType = 8; int radius = 3;
                cv::circle( colormap, center, radius, cv::Scalar( 0, 0, 255 ), thickness, lineType );

                thickness = 1; lineType = 8;
                int length = 4; cv::Point end;
                end.x = length*sin(yaw) + kpos.at<float>(0);
                end.y = length*cos(yaw) + kpos.at<float>(1);
                cv::line( colormap, center, end, cv::Scalar( 0, 255, 0 ), thickness, lineType );

                //cv::Mat zoomcolor = cv::Mat::zeros(480, 640, CV_8UC3);
                //cv::resize(colormap(roi), zoomcolor, zoomcolor.size(), 0, 0, cv::INTER_CUBIC);

                cv::imshow(robotName, colormap);
                //cv::imshow("COLOR zoom", zoomcolor);
                cv::imshow(povName, pov);
                cv::waitKey(15);
        }

        return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
        // Init the ROS node
        ros::init(argc, argv, "top_pov2map_srv");

        ROS_INFO("Starting top_pov2map_srv application ...");

        // Precondition: Valid clock
        ros::NodeHandle nh, nh2, nh3, nh4;
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
                cv::namedWindow(robotName, cv::WINDOW_AUTOSIZE);
                cv::namedWindow(povName, cv::WINDOW_AUTOSIZE);
        }

        ros::ServiceServer service = nh.advertiseService("/brl_hearts/navigation/top_pov2map_srv", srvCallback);

        ROS_INFO_STREAM("Subscribing to /brl_hearts/navigation/mapping_pose_srv ...");
        client_pose = nh2.serviceClient<hearts_nav_msgs::GetRobotPose>("/brl_hearts/navigation/mapping_pose_srv");

        ROS_INFO_STREAM("Subscribing to /brl_hearts/navigation/top_pov_srv ...");
        client_pov = nh3.serviceClient<hearts_nav_msgs::GetProjectedView>("/brl_hearts/navigation/top_pov_srv");

        ROS_INFO_STREAM("Subscribing to /brl_hearts/navigation/top_map_srv ...");
        client_map = nh4.serviceClient<hearts_nav_msgs::GetOccupancyMap>("/brl_hearts/navigation/top_map_srv");

        //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
        ros::Rate loop_rate(5);

        ros::spin();

        return EXIT_SUCCESS;
}
