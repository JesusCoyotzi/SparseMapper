/// @brief Copyright (C) 2016 Toyota Motor Corporatio
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");

  // initalize ROS publisher
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("/hsrb/head_trajectory_controller/command", 10);

  // wait to establish connection between the controller
  while (pub.getNumSubscribers() == 0) {
    ros::Duration(0.1).sleep();
  }

  // make sure the controller is running
  ros::ServiceClient client = n.serviceClient<controller_manager_msgs::ListControllers>(
      "/hsrb/controller_manager/list_controllers");
  controller_manager_msgs::ListControllers list_controllers;
  bool running = false;
  while (running == false) {
    ros::Duration(0.1).sleep();
    if (client.call(list_controllers)) {
      for (unsigned int i = 0; i < list_controllers.response.controller.size(); i++) {
        controller_manager_msgs::ControllerState c = list_controllers.response.controller[i];
        if (c.name == "head_trajectory_controller" && c.state == "running") {
          running = true;
        }
      }
    }
  }

  // fill ROS message
  trajectory_msgs::JointTrajectory traj;

  traj.joint_names.push_back("head_pan_joint");
  traj.joint_names.push_back("head_tilt_joint");

  traj.points.resize(1);

  traj.points[0].positions.resize(2);
  traj.points[0].positions[0] = 0.5;
  traj.points[0].positions[1] = 0.5;
  traj.points[0].velocities.resize(2);
  for (size_t i = 0; i < 2; ++i) {
    traj.points[0].velocities[i] = 0.0;
  }
  traj.points[0].time_from_start = ros::Duration(3.0);

  // publish ROS message
  pub.publish(traj);

  ros::spinOnce();

  return 0;
}
