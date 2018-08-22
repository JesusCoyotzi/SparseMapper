#include "hearts_tools/transformations.h"

bool Transformations::is_node_set = false;
tf::TransformListener* Transformations::tf_listener;
int Transformations::counter = 0;

bool Transformations::setNodeHandle(ros::NodeHandle* nh)
{
    std::cout << "JustinaTools.->Setting ros node..." << std::endl;
    tf_listener = new tf::TransformListener();
    tf_listener->waitForTransform("map", "laser_link", ros::Time(0), ros::Duration(10.0));
    tf_listener->waitForTransform("base_link", "left_arm_link1", ros::Time(0), ros::Duration(10.0));
    tf_listener->waitForTransform("base_link", "right_arm_link1", ros::Time(0), ros::Duration(10.0));
}

//PointCloud Msg to CvMat
void Transformations::PointCloud2Msg_ToCvMat(sensor_msgs::PointCloud2& pc_msg, cv::Mat& bgr_dest, cv::Mat& pc_dest)
{
	//std::cout << "ObjectDetectorNode.-> Transforming from PointCloud2 ros message to cv::Mat type" << std::endl;
	//std::cout << "ObjectDetectorNode.-> Width= " << pc_msg.width << "  height= " << pc_msg.height << std::endl;
	pcl::PointCloud<pcl::PointXYZRGBA> pc_pcl;
	pcl::fromROSMsg(pc_msg, pc_pcl);  //Transform from PointCloud2 msg to pointCloud (from pcl) type

	if(!pc_pcl.isOrganized())
	{
		std::cout << "ObjectDetectorNode.->Point cloud is not organized!! O M G!!!!" << std::endl;
		return;
	}
	//std::cout << "ObjectDetectorNode.-> Pcl_w= " << pc_pcl.width << "  pcl_h= " << pc_pcl.height << std::endl;
	bgr_dest = cv::Mat::zeros(pc_pcl.height, pc_pcl.width, CV_8UC3);
	pc_dest = cv::Mat::zeros(pc_pcl.height, pc_pcl.width, CV_32FC3);
	
	//pcl::PointXYZRGBA p_ = pc_pcl.at(320, 240);
	//std::cout<<"ObjectDetectorNode: Middle point: "<<p_.x << " " <<p_.y <<" "<<p_.z <<" "<< p_.r<<" "<<p_.g<<" "<< p_.b << std::endl;
	for (int h=0; h<bgr_dest.rows; h++)
		for (int w=0; w<bgr_dest.cols; w++)
		{
			pcl::PointXYZRGBA p = pc_pcl.at(w, h);

			bgr_dest.data[h*bgr_dest.step + w*3] = (unsigned char)p.b;
			bgr_dest.data[h*bgr_dest.step + w*3 + 1] = (unsigned char)p.g;
			bgr_dest.data[h*bgr_dest.step + w*3 + 2] = (unsigned char)p.r;
			pc_dest.at<cv::Vec3f>(h,w)[0] = isnan(p.x) ? 0.0 : p.x;
			pc_dest.at<cv::Vec3f>(h,w)[1] = isnan(p.y) ? 0.0 : p.y;
			pc_dest.at<cv::Vec3f>(h,w)[2] = isnan(p.z) ? 0.0 : p.z;
		}

}

void Transformations::PointCloud2Msg_ToCvMat(const sensor_msgs::PointCloud2::ConstPtr& pc_msg, cv::Mat& bgr_dest, cv::Mat& pc_dest)
{
	pcl::PointCloud<pcl::PointXYZRGB> pc_pcl;
	pcl::fromROSMsg(*pc_msg, pc_pcl);  //Transform from PointCloud2 msg to pointCloud (from pcl) type

	if(!pc_pcl.isOrganized())
	{
		std::cout << "ObjectDetectorNode.->Point cloud is not organized!! O M G!!!!" << std::endl;
		return;
	}
	//std::cout << "ObjectDetectorNode.-> Pcl_w= " << pc_pcl.width << "  pcl_h= " << pc_pcl.height << std::endl;
	bgr_dest = cv::Mat::zeros(pc_pcl.height, pc_pcl.width, CV_8UC3);
	pc_dest = cv::Mat::zeros(pc_pcl.height, pc_pcl.width, CV_32FC3);
	
	//pcl::PointXYZRGB p_ = pc_pcl.at(320, 240);
	//std::cout<<"ObjectDetectorNode: Middle point: "<<p_.x << " " <<p_.y <<" "<<p_.z <<" "<< p_.r<<" "<<p_.g<<" "<< p_.b << std::endl;
	for (int h=0; h<bgr_dest.rows; h++)
		for (int w=0; w<bgr_dest.cols; w++)
		{
			pcl::PointXYZRGB p = pc_pcl.at(w,h);
			bgr_dest.data[h*bgr_dest.step + w*3] = (unsigned char)p.b;
			bgr_dest.data[h*bgr_dest.step + w*3 + 1] = (unsigned char)p.g;
			bgr_dest.data[h*bgr_dest.step + w*3 + 2] = (unsigned char)p.r;
			pc_dest.at<cv::Vec3f>(h,w)[0] = isnan(p.x) ? 0.0 : p.x;
			pc_dest.at<cv::Vec3f>(h,w)[1] = isnan(p.y) ? 0.0 : p.y;
			pc_dest.at<cv::Vec3f>(h,w)[2] = isnan(p.z) ? 0.0 : p.z;
		}

}

//one channel int CvMat to Msg
void Transformations::CvMat_ToVectMsg(cv::Mat matmap, uint32_t* &vecmap, int &w, int &h)
{
	w = matmap.cols;
	h = matmap.rows;
	vecmap = new uint32_t[w*h];

	int idx = 0;
	for(int x = 0; x < matmap.cols; x++)
	  for(int y = 0; y < matmap.rows; y++)
	  {
		  vecmap[idx++] = (uint32_t)matmap.at<uchar>(y,x);
	  }
}

void Transformations::CvMat_ToVectMsg(cv::Mat matmap, std::vector<uint32_t>* vecmap, int &w, int &h)
{
	w = matmap.cols;
	h = matmap.rows;

	for(int x = 0; x < matmap.cols; x++)
	  for(int y = 0; y < matmap.rows; y++)
	  {
		  vecmap->push_back((uint32_t)matmap.at<uchar>(y,x));
	  }
}

void Transformations::VectMsg_ToCvMat(uint32_t* vecmap, int w, int h, cv::Mat& matmap)
{
	matmap = cv::Mat::zeros(h, w, CV_8UC1);

	int idx = 0;
	for(int x = 0; x < matmap.cols; x++)
	  for(int y = 0; y < matmap.rows; y++)
	  {
		  matmap.at<uchar>(y,x) = (uchar)vecmap[idx++];
	  }
}

void Transformations::VectMsg_ToCvMat(std::vector<uint32_t> vecmap, int w, int h, cv::Mat& matmap)
{
	matmap = cv::Mat::zeros(h, w, CV_8UC1);

	int idx = 0;
	for(int x = 0; x < matmap.cols; x++)
	  for(int y = 0; y < matmap.rows; y++)
	  {
		  matmap.at<uchar>(y,x) = (uchar)vecmap[idx++];
	  }
}

//three channel int CvMat to Msg
void Transformations::CvMatC3_ToVectMsg(cv::Mat matmap, uint32_t* &vecmap, int &w, int &h)
{
	w = matmap.cols;
	h = matmap.rows;
	vecmap = new uint32_t[w*h*3];

	int idx = 0;
	for(int x = 0; x < matmap.cols; x++)
	  for(int y = 0; y < matmap.rows; y++)
	  {
		  vecmap[idx++] = (uint32_t)matmap.at<cv::Vec3b>(y,x)[0];
		  vecmap[idx++] = (uint32_t)matmap.at<cv::Vec3b>(y,x)[1];
		  vecmap[idx++] = (uint32_t)matmap.at<cv::Vec3b>(y,x)[2];
	  }
}

void Transformations::CvMatC3_ToVectMsg(cv::Mat matmap, std::vector<uint32_t>* vecmap, int &w, int &h)
{
	w = matmap.cols;
	h = matmap.rows;

	for(int x = 0; x < matmap.cols; x++)
	  for(int y = 0; y < matmap.rows; y++)
	  {
		  vecmap->push_back((uint32_t)matmap.at<cv::Vec3b>(y,x)[0]);
		  vecmap->push_back((uint32_t)matmap.at<cv::Vec3b>(y,x)[1]);
		  vecmap->push_back((uint32_t)matmap.at<cv::Vec3b>(y,x)[2]);
	  }
}

void Transformations::VectMsg_ToCvMatC3(uint32_t* vecmap, int w, int h, cv::Mat& matmap)
{
	matmap = cv::Mat::zeros(h, w, CV_8UC3);

	int idx = 0;
	for(int x = 0; x < matmap.cols; x++)
	  for(int y = 0; y < matmap.rows; y++)
	  {
		  matmap.at<cv::Vec3b>(y,x)[0] = (uchar)vecmap[idx++];
		  matmap.at<cv::Vec3b>(y,x)[1] = (uchar)vecmap[idx++];
		  matmap.at<cv::Vec3b>(y,x)[2] = (uchar)vecmap[idx++];
	  }
}

void Transformations::VectMsg_ToCvMatC3(std::vector<uint32_t> vecmap, int w, int h, cv::Mat& matmap)
{
	matmap = cv::Mat::zeros(h, w, CV_8UC3);

	int idx = 0;
	for(int x = 0; x < matmap.cols; x++)
	  for(int y = 0; y < matmap.rows; y++)
	  {
		  matmap.at<cv::Vec3b>(y,x)[0] = (uchar)vecmap[idx++];
		  matmap.at<cv::Vec3b>(y,x)[1] = (uchar)vecmap[idx++];
		  matmap.at<cv::Vec3b>(y,x)[2] = (uchar)vecmap[idx++];
	  }
}

//double CvMat to Msg
void Transformations::CvMatf_ToVectMsg(cv::Mat matmap, float_t* &vecmap, int &w, int &h)
{
	w = matmap.cols;
	h = matmap.rows;
	vecmap = new float_t[w*h];

	int idx = 0;
	for(int x = 0; x < matmap.cols; x++)
	  for(int y = 0; y < matmap.rows; y++)
	  {
		  vecmap[idx++] = (float_t)matmap.at<float>(y,x);
	  }
}

void Transformations::CvMatf_ToVectMsg(cv::Mat matmap, std::vector<float_t>* vecmap, int &w, int &h)
{
	w = matmap.cols;
	h = matmap.rows;

	for(int x = 0; x < matmap.cols; x++)
	  for(int y = 0; y < matmap.rows; y++)
	  {
		  vecmap->push_back((float_t)matmap.at<float>(y,x));
	  }
}

void Transformations::VectMsg_ToCvMatf(float_t* vecmap, int w, int h, cv::Mat& matmap)
{
	matmap = cv::Mat::zeros(h, w, CV_32FC1);

	int idx = 0;
	for(int x = 0; x < matmap.cols; x++)
	  for(int y = 0; y < matmap.rows; y++)
	  {
		  matmap.at<float>(y,x) = (float)vecmap[idx++];
	  }
}

void Transformations::VectMsg_ToCvMatf(std::vector<float_t> vecmap, int w, int h, cv::Mat& matmap)
{
	matmap = cv::Mat::zeros(h, w, CV_32FC1);

	int idx = 0;
	for(int x = 0; x < matmap.cols; x++)
	  for(int y = 0; y < matmap.rows; y++)
	  {
		  matmap.at<float>(y,x) = (float)vecmap[idx++];
	  }
}

