#include "map_serializer.h"
#include "ros/duration.h"
#include <ros/package.h>

int main(int argc, char **argv) {
        ros::init(argc, argv, "map_serializer");
        if(argc!=2)
        {
          printf("Usage: program filename.txt\n");
          return -1;
        }
        std::string outFileName=ros::package::getPath("map_serializer")+"/maps/"+"serialized/"+argv[1];
        std::cout << "Saving to: " <<outFileName<< '\n';
        ros::NodeHandle nh_priv("~");
        map_serializer ms(outFileName);
        //ros::Rate loop(10);
        ros::Duration(1).sleep();
        if(ros::ok())
        {
                //loop.sleep();

                ros::spinOnce();
        }
        return 0;
}
