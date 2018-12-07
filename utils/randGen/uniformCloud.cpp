#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char  *argv[]) {
        if (argc<5) {
          std::cout << "Error: Usage: program filename points min max" << '\n';
          return -1;
        }

        std::string filename(argv[1]);
        int cloudSize = atoi(argv[2]);
        float minVal = atof(argv[3]);
        float maxVal = atof(argv[4]);

        pcl::PointCloud<pcl::PointXYZ>::Ptr uniformCloud (new pcl::PointCloud<pcl::PointXYZ>);
        uniformCloud->width = cloudSize;
        uniformCloud->height = 1;
        uniformCloud->points.resize(cloudSize);
        std::cout << "Generating a cloud of " << uniformCloud->width <<  " Points";

        for (int i = 0; i < cloudSize; i++) {
          //Flat cloud
          float x = rand() / (RAND_MAX+1.0f);
          float y = rand() / (RAND_MAX+1.0f);
          uniformCloud->points[i].x = (maxVal - minVal)*x + minVal;
          uniformCloud->points[i].y = (maxVal - minVal)*y + minVal;
          //uniformCloud->points[i].z = 1024 * rand() / (RAND_MAX+1.0f);
          uniformCloud->points[i].z = 0;
        }

        if (pcl::io::savePCDFileASCII(filename, *uniformCloud) == -1) //* load the file
        {
                PCL_ERROR ("Couldn't write file\n");
                return (-1);
        }


        std::cout << "Saved at " << filename <<'\n';

        return 0;
}
