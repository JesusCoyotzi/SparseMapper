#include "preprocessing.h"

//TODO Adding some statistical outlier removal before subsampling would be nice
//http://pointclouds.org/documentation/tutorials/statistical_outlier.php
cloudPreprocessor::cloudPreprocessor(ros::NodeHandle &nh)
{
        nh_ = nh;
        ros::NodeHandle nh_priv("~");
        nh_priv.param<std::string>("baseFrame",baseFrame,"base_link");
        nh_priv.param<float>("crop_max_distance",cropMaxDistance,10);
        nh_priv.param<float>("crop_min_distance",cropMinDistance,-0.1);
        nh_priv.param<float>("voxelSize",voxelSize,0.01); //in meters

        inCloudSub = nh_.subscribe<sensor_msgs::PointCloud2>("/cloud",10,&cloudPreprocessor::processCallback,this);
        processedCloudPub=nh_.advertise<sensor_msgs::PointCloud2>
                                   ("/processed_cloud",2);

        std::cout << "Starting Cloud preprocessor by Coyo-Soft" << '\n';
}
void cloudPreprocessor::processCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::fromROSMsg(*input, *cloud);

        // std::cout << "With fields:" << '\n';
        // for (int i = 0; i < input->fields.size(); i++) {
        //   std::cout << input->fields[i].name << '\n';
        // }
        if (cloud->points.size()<1) {
                std::cout << "Error empty cloud received skiped" << '\n';
                return;
        }

        if (cropMaxDistance > 0) {
                //  std::cout << "Z filter" << '\n';
                filterDistanceZ(cloud);
        }
        if (voxelSize > 0) {
                //std::cout << "Voxel filter" << '\n';
                voxelFilter(voxelSize,cloud);
        }
        if (!transformCloud(cloud))
        {
                std::cout << "Cloud not transform cloud skipping" << '\n';
                return;
        }
        //convert to msg and publish
        std::cout << "Got PointCloud @ frame " << cloud->header.frame_id <<'\n';
        std::cout << "With size " <<  cloud->points.size() <<"\n";
        std::cout << "Publishing processed cloud" << '\n';
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud,cloud_msg);
        processedCloudPub.publish(cloud_msg);
        return;
}

void cloudPreprocessor::filterDistanceZ(cloudRGBAPtr cloud)
{
        //removes all points below z (depth)
        //done in the point cloud frame to crop at depth
        pcl::PassThrough<pcl::PointXYZRGBA> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(cropMinDistance, cropMaxDistance);
        //pass.setFilterLimitsNegative (true);
        pass.filter(  *cloud);
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
        std::cout << "Transforming cloud to " << baseFrame<<'\n';
        try{
                sux = pcl_ros::transformPointCloud(baseFrame,
                                                   *cloud,
                                                   *cloud,
                                                   tf_listener
                                                   );
        }
        catch (tf::TransformException ex) {
                std::cout << "Erroring!!!!!" << '\n';
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();

        }
        return sux;
}
