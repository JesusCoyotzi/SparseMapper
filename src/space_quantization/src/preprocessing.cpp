#include "preprocessing.h"

cloudPreprocessor::cloudPreprocessor(ros::NodeHandle &nh)
{
        nh_ = nh;
        ros::NodeHandle nh_priv("~");
        nh_priv.param<std::string>("baseFrame",baseFrame,"base_link");
        nh_priv.param<float>("cropDistance",cropDistance,10);
        nh_priv.param<float>("voxelSize",voxelSize,0.01); //in meters
        nh_priv.param<bool>("cropEnable",cropEnable,false);
        nh_priv.param<bool>("voxEnable",voxEnable,true);


        inCloudSub = nh_.subscribe<sensor_msgs::PointCloud2>("/cloud",10,&cloudPreprocessor::processCallback,this);
        processedCloudPub=nh_.advertise<sensor_msgs::PointCloud2>
                                   ("/processed_cloud",2);

        std::cout << "Starting Cloud preprocessor by Coyo-Soft" << '\n';
}
void cloudPreprocessor::processCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::fromROSMsg(*input, *cloud);
        std::cout << "Got PointCloud @ frame " << cloud->header.frame_id <<'\n';
        if (cropEnable) {
          filterDistanceZ(cropDistance,cloud);
          
        }
        if (voxEnable) {
          voxelFilter(voxelSize,cloud);

        }
        transformCloud(cloud);
        //convert to msg and publish
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud,cloud_msg);
        processedCloudPub.publish(cloud_msg);
}

void cloudPreprocessor::filterDistanceZ(double maxDist, cloudRGBAPtr cloud)
{
        //removes all points below z (depth)
        //done in the point cloud frame to crop at depth
        pcl::PassThrough<pcl::PointXYZRGBA> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0, maxDist);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*cloud);
        return;
}

void cloudPreprocessor::voxelFilter(double vSize,cloudRGBAPtr cloud)
{
  //Simple voxel filtering object
  pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (vSize, vSize, vSize);
  sor.filter (*cloud);
}

bool cloudPreprocessor::transformCloud(cloudRGBAPtr cloud)
{
  bool sux = false;
  tf::StampedTransform transformTf;
  try{
          sux = pcl_ros::transformPointCloud(baseFrame,
                                             *cloud,
                                             *cloud,
                                             tf_listener
                                             );
  }
  catch (tf::TransformException ex) {
          //std::cout << "Erroring!!!!!" << '\n';
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
  }
  return sux;
}
