#include "sparse_map_utils/cloudSimulation.h"

int main(int argc, char  *argv[]) {

        ros::init(argc,argv,"cloud_simulation");
        ros::NodeHandle nh;
        cloudSimulation cS(nh);
        if (!cS.loadCloud())
        {
                std::cout << "FAILED" << '\n';
                return -1;
        }
        ros::Duration(2.0).sleep();
        cS.startMonteCarlo();
        ros::spin();
        return 0;


}
