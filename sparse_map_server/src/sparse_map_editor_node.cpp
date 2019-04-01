#include "sparse_map_server/sparse_map_editor.h"

int main(int argc, char *argv[])
{
        ros::init(argc,argv,"sparse_map_editor");
        ros::NodeHandle nh;
        std::cout << "Starting map editor by CoyoSoft" << '\n';
        sparseMapEditor sme(nh);
        ros::spin();
        return 0;
}
