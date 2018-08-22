// C++ standard headers
#include <exception>
#include <string>
#include <math.h>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <ros/topic.h>

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// Hearts headers
#include <hearts_nav_msgs/RobotPose.h>
#include <hearts_nav_msgs/GetRobotPose.h>

#include <hearts_tools/transformations.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Global parameters
bool debug = false;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
        //Server parameters
        hearts_nav_msgs::GetRobotPose srv;
        ///////////

        // Init the ROS node
        ros::init(argc, argv, "mapping_pose_cli");

        ROS_INFO("Starting mapping_pose_cli application ...");

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

        ROS_INFO_STREAM("Subscribing to /brl_hearts/navigation/mapping_pose_srv ...");
        ros::ServiceClient client = nh.serviceClient<hearts_nav_msgs::GetRobotPose>("/brl_hearts/navigation/mapping_pose_srv");

        ros::Rate loop(10);
        while(ros::ok() && cv::waitKey(15) != 27)
        {
                if (client.call(srv))
                {
                        if (srv.response.robot_pose.id < 0)
                                return 1;

                        if ( (srv.response.robot_pose.o_w * srv.response.robot_pose.o_h) == 0)
                                return 1;

                        cv::Mat o, pos, k, rt;
                        double yaw;
                        int w, h;

                        w = srv.response.robot_pose.o_w;
                        h = srv.response.robot_pose.o_h;
                        Transformations::VectMsg_ToCvMatf(srv.response.robot_pose.o, w, h, o);

                        w = srv.response.robot_pose.pos_w;
                        h = srv.response.robot_pose.pos_h;
                        Transformations::VectMsg_ToCvMatf(srv.response.robot_pose.pos, w, h, pos);

                        yaw = srv.response.robot_pose.yaw;

                        w = srv.response.robot_pose.k_w;
                        h = srv.response.robot_pose.k_h;
                        Transformations::VectMsg_ToCvMatf(srv.response.robot_pose.k, w, h, k);

                        w = srv.response.robot_pose.rt_w;
                        h = srv.response.robot_pose.rt_h;
                        Transformations::VectMsg_ToCvMatf(srv.response.robot_pose.rt, w, h, rt);

                        if (debug)
                        {
                                ///Display current robot pose
                                std::cout << "\n o" << o << "\n pos" << pos << "\n yaw" << yaw << "\n K" << k << "\n Rt" << rt << std::endl;
                        }
                } else
                {
                        ROS_ERROR("Failed to call service: top_pov2map_srv");
                        return 1;
                }

                ros::spinOnce();
                loop.sleep();
        }

        return EXIT_SUCCESS;
}
