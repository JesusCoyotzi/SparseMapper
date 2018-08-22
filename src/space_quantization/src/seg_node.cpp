#include "space_seg.h"
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>

//#include <string>


int main(int argc, char *argv[])
{
        ros::init(argc,argv,"seg_node");
        ros::NodeHandle nh;
        spaceSegmenter ss(nh);

        ros::spin();
        return 0;
}
