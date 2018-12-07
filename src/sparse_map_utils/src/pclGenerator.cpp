#include "sparse_map_utils/pclGenerator.h"

cloudGenerator::cloudGenerator(ros::NodeHandle &nh)
{
        nh_=nh;
        nh_priv = ros::NodeHandle("~");
        nh_priv.param<std::string>("pcd_file",pcdFile,"cloud.pcd");
        cloud  = new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcdFile, *cloud) == -1) //* load the file
        {
                PCL_ERROR ("Couldn't read file\n");
                return (-1);
        }

        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>
                                  ("out_cloud",1);
        return;
}
