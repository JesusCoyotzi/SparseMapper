#include "sparse_map_utils/cloudSimulation.h"

int main(int argc, char  *argv[]) {

        ros::init(argc,argv,"cloud_simulation");
        ros::NodeHandle nh;
        cloudSimulation cS(nh);
        if (!cS.loadCloud())
        {
                return -1;
        }
        cS.startMonteCarlo();
        ros::spin();
        return 0;


}
