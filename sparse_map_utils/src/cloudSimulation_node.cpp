#include "sparse_map_utils/cloudSimulation.h"

int main(int argc, char  *argv[]) {

        ros::init(argc,argv,"cloud_simulation");
        ros::NodeHandle nh;
        cloudSimulation cS(nh);
        ros::Rate l(0.5);
        if (cS.getCloudFiles()<1)
        {
                std::cout << "FAILED" << '\n';
                return -1;
        }
        std::cout << "Starting simulations" << '\n';
        while (cS.loadNextCloud()) {
                cS.monteCarlo();
                ros::spinOnce();
                l.sleep();
        }
        //ros::Duration(2.0).sleep();
        //cS.startMonteCarlo();
        return 0;


}
