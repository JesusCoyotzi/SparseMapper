#include "preprocessing.h"

int main(int argc, char *argv[]) {
  ros::init(argc,argv,"cloud_preprocessor");
  ros::NodeHandle nh;
  cloudPreprocessor cP(nh);
  ros::spin();
  return 0;
}
