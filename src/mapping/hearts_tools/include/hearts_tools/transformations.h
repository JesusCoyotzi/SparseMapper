#ifndef TRANSFORMATIONS_H
#define TRANSFORMATIONS_H

//#pragma once
#include <iostream>
#include <vector>
#include <cmath>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"

#include "sensor_msgs/PointCloud2.h"
#include "hearts_nav_msgs/ProjectedView.h"
#include "pcl_conversions/pcl_conversions.h"

#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"

//#include "sensor_msgs/LaserScan.h"

#include <sstream> 
#include <string>

class Transformations
{
private:
	static bool is_node_set;
	static tf::TransformListener* tf_listener;
	static int counter;

public:
	static bool setNodeHandle(ros::NodeHandle* nh);

	//PointCloud Msg to CvMat
	static void PointCloud2Msg_ToCvMat(sensor_msgs::PointCloud2& pc_msg, cv::Mat& bgr_dest, cv::Mat& pc_dest);
	static void PointCloud2Msg_ToCvMat(const sensor_msgs::PointCloud2::ConstPtr& pc_msg, cv::Mat& bgr_dest, cv::Mat& pc_dest);
	
	//one channel int CvMat to Msg
	static void CvMat_ToVectMsg(cv::Mat matmap, uint32_t* &vecmap, int &w, int &h);
	static void CvMat_ToVectMsg(cv::Mat matmap, std::vector<uint32_t>* vecmap, int &w, int &h);
	
	static void VectMsg_ToCvMat(uint32_t* vecmap, int w, int h, cv::Mat& matmap);
	static void VectMsg_ToCvMat(std::vector<uint32_t> vecmap, int w, int h, cv::Mat& matmap);

	//three channel int CvMat to Msg
	static void CvMatC3_ToVectMsg(cv::Mat matmap, uint32_t* &vecmap, int &w, int &h);
	static void CvMatC3_ToVectMsg(cv::Mat matmap, std::vector<uint32_t>* vecmap, int &w, int &h);
	
	static void VectMsg_ToCvMatC3(uint32_t* vecmap, int w, int h, cv::Mat& matmap);
	static void VectMsg_ToCvMatC3(std::vector<uint32_t> vecmap, int w, int h, cv::Mat& matmap);

	//float CvMat to Msg
	static void CvMatf_ToVectMsg(cv::Mat matmap, float_t* &vecmap, int &w, int &h);
	static void CvMatf_ToVectMsg(cv::Mat matmap, std::vector<float_t>* vecmap, int &w, int &h);
	
	static void VectMsg_ToCvMatf(float_t* vecmap, int w, int h, cv::Mat& matmap);
	static void VectMsg_ToCvMatf(std::vector<float_t> vecmap, int w, int h, cv::Mat& matmap);
};

#endif

