#include "sparse_map_server/sparse_map_server.h"

int main(int argc, char *argv[])
{
        ros::init(argc,argv,"sparse_map_server");
        ros::NodeHandle nh;
        std::cout << "Starting map server by CoyoSoft" << '\n';
        sparseMapServer sms(nh);
        ros::spin();
        return 0;
}
