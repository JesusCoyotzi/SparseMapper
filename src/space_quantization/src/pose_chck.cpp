#include "pose_chck.h"

poseChck::poseChck(ros::NodeHandle &nh)
{
        nh_=nh;
        ros::NodeHandle nh_priv("~");
        nh_priv.param<float>("angular_update",angularUpdate,90);
        angularUpdate *=(M_PI/180.0);
        nh_priv.param<float>("linear_update",linearUpdate,0.50);

        locSub= nh_.subscribe("/localization_pose",1,&poseChck::checkAdvanceCov,this);
        slamSub= nh_.subscribe("/slam_pose",1,&poseChck::checkAdvance,this);
        cloudSub = nh_.subscribe("/cloud_in",1,&poseChck::cloudCallback,this);
        cloudPub = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_out",1);
        previousPose.header.stamp = ros::Time::now();
        previousPose.header.frame_id = "map";
        previousPose.pose.position.x = 0;
        previousPose.pose.position.y = 0;
        previousPose.pose.position.z = 0;
        previousPose.pose.orientation.x = 0;
        previousPose.pose.orientation.y = 0;
        previousPose.pose.orientation.z = 0;
        previousPose.pose.orientation.w = 1;
        publishCloud = true;
        std::cout << "*[-Started pose checker by Coyo-Soft-]*" << '\n';
        std::cout << "Angular update is: " << angularUpdate *180/M_PI<<'\n';

}

void poseChck::checkAdvanceCov(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
        geometry_msgs::Pose currentPose = msg.pose.pose;
        std_msgs::Header head =msg.header;
        double dis = linearDistance(currentPose, previousPose.pose);
        // std::cout << "Current pose :" << '\n';
        // std::cout << currentPose << '\n';
        // std::cout << "previousPose" << '\n';
        // std::cout << previousPose << '\n';
        if (dis > linearUpdate) {
                std::cout << "Distance threshold exceeded: " << dis<<'\n';
                previousPose.pose = currentPose;
                previousPose.header = head;
                publishCloud = true;
                return;
        }
        double angle = angularDistance(currentPose, previousPose.pose);
        if (angle > angularUpdate) {
                std::cout << "Angular threshold exceeded: " << angle*180/M_PI<<'\n';
                previousPose.pose = currentPose;
                previousPose.header = head;
                publishCloud = true;
        }
        return;
}

void poseChck::checkAdvance(const geometry_msgs::PoseStamped &msg)
{
        geometry_msgs::Pose currentPose = msg.pose;
        std_msgs::Header head =msg.header;
        double dis = linearDistance(currentPose, previousPose.pose);
        // std::cout << "Current pose :" << '\n';
        // std::cout << currentPose << '\n';
        // std::cout << "previousPose" << '\n';
        // std::cout << previousPose << '\n';
        if (dis > linearUpdate) {
                std::cout << "Distance threshold exceeded: " << dis<<'\n';
                previousPose.pose = currentPose;
                previousPose.header = head;
                publishCloud = true;
                return;
        }
        double angle = angularDistance(currentPose, previousPose.pose);
        if (angle > angularUpdate) {
                std::cout << "Angular threshold exceeded: " << angle*180/M_PI<<'\n';
                previousPose.pose = currentPose;
                previousPose.header = head;
                publishCloud = true;
        }
        return;
}

void poseChck::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
        if (publishCloud) {
                sensor_msgs::PointCloud2 cloud = *msg;
                cloud.header.frame_id = msg->header.frame_id;
                cloud.header.stamp = msg->header.stamp;
                std::cout << "Publish Cloud!! @ frame" <<  cloud.header.frame_id<<'\n';
                cloudPub.publish(cloud);
                publishCloud=false;
        }
}

double poseChck::linearDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
        double x1 = p1.position.x;
        double y1 = p1.position.y;
        double z1 = p1.position.z;
        double x2 = p2.position.x;
        double y2 = p2.position.y;
        double z2 = p2.position.z;
        return sqrtf((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2));
}

double poseChck::angularDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
        double angle1 =2* acos(p1.orientation.w);
        double angle2 =2* acos(p2.orientation.w);
        return fabs(angle2-angle1);
}
