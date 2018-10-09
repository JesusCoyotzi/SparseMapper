#include "quantization_utilities/tf_saver.h"

int main(int argc, char  *argv[]) {
        ros::init(argc,argv,"tf_saver");
        ros::NodeHandle nh;
        if (argc != 3) {
                std::cout << "Usage: node origin target" << '\n';
                exit(-1);
        }

        std::string origin(argv[1]);
        std::string target(argv[2]);
        tfSaver ts(nh);
        bool done=false;
        ros::Rate sr(5);

        while (ros::ok() && !done)
        {
                ros::spinOnce();
                if(ts.getTransform(origin,target))
                {
                  done = ts.saveTransform();
                }
                sr.sleep();
        }
        return 0;
}
