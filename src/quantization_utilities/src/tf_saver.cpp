#include "quantization_utilities/tf_saver.h"

tfSaver::tfSaver(ros::NodeHandle &nh)
{
        nh_=nh;
        std::cout << "Starting TF saver by CoyoSoft" << '\n';
        ros::NodeHandle nh_priv("~");
        nh_priv.param<std::string>("tf_file",tfFile,"tf.csv");
}

bool tfSaver::getTransform(std::string frame_orig, std::string frame_target)
{
        bool suxess = true;
        try{
                listener.lookupTransform(frame_orig, frame_target,
                                         ros::Time(0), transform);
        }
        catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                suxess = false;
        }
        return suxess;

}

bool tfSaver::saveTransform()
{
        bool sux = true;
        std::cout << "Writing TF to:" << tfFile<< '\n';
        std::ofstream tfStream(tfFile.c_str(), std::ios::out);
        if (tfStream.is_open()) {
                tfStream << "Parent id: " << transform.frame_id_ << "\n";
                tfStream << "Child id: " << transform.child_frame_id_ << "\n";
                tf::Vector3 trans = transform.getOrigin();
                tfStream<<"Translation: "
                        << trans.x()<< " "
                        << trans.y()<< " "
                        << trans.z()<< "\n";
                tf::Quaternion Q =transform.getRotation();
                tfStream<<"Quaternion: "
                        << Q.x()<<" "
                        << Q.y()<< " "
                        << Q.z()<< " "
                        << Q.w()<< "\n";
                tfStream.close();
        }
        else
        {
          std::cout << "Error writing to file" << '\n';
          sux =false;
        }
        return sux;
}
