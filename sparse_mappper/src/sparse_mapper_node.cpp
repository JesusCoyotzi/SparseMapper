#include "sparse_mapper/sparse_mapper.h"
int main(int argc, char *argv[])
{
        ros::init(argc,argv,"sparse_mapper_node");
        ros::NodeHandle nh;
        sparseMapper sm(nh);
        //sm.setVQMapCallbacks();
        ros::spin();
        return 0;
}
