#include "space_separation.h"
int main(int argc, char *argv[])
{
        ros::init(argc,argv,"sep_node");
        ros::NodeHandle nh;
        spaceSeparator ss(nh);

        ros::spin();
        return 0;
}
