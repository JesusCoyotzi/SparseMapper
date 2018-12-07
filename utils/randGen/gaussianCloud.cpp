#include <iostream>
#include <random>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char  *argv[]) {
        if (argc<3) {
                std::cout << "Error: Usage: program filename points [mean_x mean_y mean_z, dev_x, dev_y, dev_z]" << '\n';
                return -1;
        }
        float meanX=0, meanY=0, meanZ=0;
        float sigmaX=1, sigmaY=1, sigmaZ=1;
        if (argc > 3) {
                meanX=atof(argv[3]);
                meanY=atof(argv[4]);
                meanZ=atof(argv[5]);
                sigmaX=atof(argv[6]);
                sigmaY=atof(argv[7]);
                sigmaZ=atof(argv[8]);
        }

        std::string filename(argv[1]);
        int cloudSize = atoi(argv[2]);

        pcl::PointCloud<pcl::PointXYZ>::Ptr gaussCloud (new pcl::PointCloud<pcl::PointXYZ>);
        gaussCloud->width = cloudSize;
        gaussCloud->height = 1;
        gaussCloud->points.resize(cloudSize);
        std::cout << "Generating a cloud of " << gaussCloud->width <<  " Points";

        std::default_random_engine generator;
        std::normal_distribution<double> distributionX(meanX,sigmaX);
        std::normal_distribution<double> distributionY(meanY,sigmaY);
        std::normal_distribution<double> distributionZ(meanZ,sigmaZ);

        for (int i = 0; i < cloudSize; i++) {
                //Flat cloud
                float x = distributionX(generator);
                float y = distributionY(generator);
                float z = distributionZ(generator);
                gaussCloud->points[i].x = x;
                gaussCloud->points[i].y = y;
                gaussCloud->points[i].z = z;

        }

        if (pcl::io::savePCDFileASCII(filename, *gaussCloud) == -1) //* load the file
        {
                PCL_ERROR ("Couldn't write file\n");
                return (-1);
        }


        std::cout << "Saved at " << filename <<'\n';

        return 0;
}
