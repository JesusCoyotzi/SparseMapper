#include "quantization_utilities/tf_reader.h"

int main(int argc, char  *argv[])
{
        ros::init(argc,argv,"tf_saver");
        ros::NodeHandle nh;
        if (argc != 2) {
                std::cout << "Usage: node filepath" << '\n';
                exit(-1);
        }
        std::string filepath(argv[1]);

        bool done=false;
        ros::Rate sr(5);

        tfReader tr(nh);
        if(tr.loadTf(filepath))
        {
                tr.latchTransform();
                ros::spin();
        }
        return 0;
}
