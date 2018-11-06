#include "pose_chck.h"

int main(int argc, char *argv[])
{
        ros::init(argc,argv,"pose_chck_node");
        ros::NodeHandle nh;
        poseChck ps(nh);
        
        ros::spin();
        return 0;
}
