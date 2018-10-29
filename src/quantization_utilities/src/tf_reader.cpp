#include "quantization_utilities/tf_reader.h"

tfReader::tfReader(ros::NodeHandle &nh)
{
        nh_=nh;
}

bool tfReader::loadTf(std::string filepath)
{
        bool success = true;
        std::ifstream tfFile(filepath.c_str(), std::ios::in);
        std::string line;
        if (tfFile.is_open()) {

                // while (std::getline(tfFile, line)) {
                //         iss.str(std::string());
                //         iss.clear();
                //         iss.str(line);
                //         std::cout << iss.str() << '\n';
                //         if(!(iss>>field))
                //         {
                //                 std::cout << "Error" << '\n';
                //         }
                //         else
                //         {
                //                 std::cout << field << '\n';
                //         }
                // }
                std::string field;
                std::getline(tfFile, line);
                std::stringstream iss(line);
                //get frames
                //Get parent id
                if (!(iss >> field>> parent_id ))
                {
                        std::cout << "Error parsing parent" << '\n';
                        success = false;
                }
                std::cout << "Frame_id: " << parent_id<< '\n';
                //get children id
                std::getline(tfFile, line);
                iss.str(std::string());
                iss.clear();
                iss.str(line);

                if (!(iss >> field>> child_id ))
                {
                        std::cout << "Error parsing child_id" << '\n';
                        success = false;
                }
                std::cout << "Children_id: " << child_id<< '\n';
                //Get trasnlation
                std::getline(tfFile, line);
                iss.str(std::string());
                iss.clear();
                iss.str(line);
                float x, y,z;
                if (!(iss >>field>>x>> y>>z ))
                {
                        std::cout << "Error parsing trasnlation" << '\n';
                        success = false;
                }
                std::cout << "Translation:" <<x <<" "
                          << y << " "
                          << z << " "<< '\n';
                //Get Quaternion
                std::getline(tfFile, line);
                iss.str(std::string());
                iss.clear();
                iss.str(line);
                float qx, qy,qz,qw;
                if (!(iss >> field>>qx>>qy>>qz>>qw ))
                {
                        std::cout << "Error parsing Quat" << '\n';
                        success = false;
                }
                std::cout << "Translation:" <<qx <<" "
                          << qy << " "
                          << qz << " "
                          << qw << '\n';
                transform.header.stamp = ros::Time::now();
                transform.header.frame_id = parent_id;
                transform.child_frame_id = child_id;
                transform.transform.translation.x=x;
                transform.transform.translation.y=y;
                transform.transform.translation.z=z;

                transform.transform.rotation.x=qx;
                transform.transform.rotation.y=qy;
                transform.transform.rotation.z=qz;
                transform.transform.rotation.w=qw;
                tfFile.close();
        }
        else{
                success =false;
                std::cout << "Error opening" << filepath<<'\n';
        }
        return success;
}

void tfReader::latchTransform()
{
        ROS_INFO("Publishing trasnform from %s to %s",parent_id.c_str(), child_id.c_str());
        static_broadcaster.sendTransform(transform);
}
