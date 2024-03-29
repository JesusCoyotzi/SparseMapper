#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
//Boost
#include "boost/filesystem.hpp"
#include <time.h>
#include <stdlib.h>

int main(int argc, char  *argv[]) {
        //Program readas and subsamples pointcloud,
        //Centers it on the centroid
        if (argc<3)
        {
                std::cout << "Error: Usage: pdcConverter filename.[obj/ply/pcd] outfile.pcd [voxelSize]" << '\n';
                return -1;
        }
        std::string pcdFile(argv[1]);
        std::cout << "Reading file " << pcdFile<<'\n';
        boost::filesystem::path p(pcdFile);
        std::string fileType(p.extension().string());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

        if (!fileType.compare(".pcd")) {
                if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdFile, *cloud) == -1)
                {
                        PCL_ERROR ("Couldn't read file  as PCD\n");
                        return 0;
                }


        }
        else if(!fileType.compare(".ply")) {
                if(pcl::io::loadPLYFile<pcl::PointXYZRGB> (pcdFile, *cloud) == -1)
                {
                        PCL_ERROR("Could not read file as PLY");
                        return -1;

                }
        }
        else if(!fileType.compare(".obj")) {
                if(pcl::io::loadOBJFile<pcl::PointXYZRGB> (pcdFile, *cloud) == -1)
                {
                        PCL_ERROR("Could not read file as OBJ");
                        return -1;

                }
        }

        std::cout << "Read cloud with " << cloud->points.size()<<" points\n";
        //Sample a few elements from the cloud for checking
        srand(time(NULL));
        for (int i = 0; i < 10; i++) {
                int idx = rand()%cloud->points.size();
                printf("Point [%d] : ", idx);
                std::cout <<cloud->points[idx] << '\n';
        }
        if (argc>3)
        {
                float leafSize = atof(argv[3]);
                pcl::VoxelGrid<pcl::PointXYZRGB> sor;
                sor.setInputCloud (cloud);
                sor.setLeafSize (leafSize, leafSize, leafSize);
                sor.filter (*cloud);
                std::cout << "Subsampling with leaf size: " << leafSize << '\n';
                std::cout << "Subsampled to: " << cloud->points.size() << '\n';
        }

        std::string outfile(argv[2]);

        //Save in binary mode to save memory and get more precision

        pcl::io::savePCDFile(outfile, *cloud,true);
        std::cerr << "Saved " << cloud->points.size () << " data points to "  << argv[2] << std::endl;
        // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
        // viewer.showCloud (cloud);
        // while (!viewer.wasStopped ())
        // {
        // }
        return 0;
}
