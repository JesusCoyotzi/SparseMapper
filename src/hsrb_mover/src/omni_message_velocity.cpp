/// @brief Copyright (C) 2016 Toyota Motor Corporation
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");

  // initalize ROS publisher
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/hsrb/command_velocity", 10);

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
        if (c.name == "omni_base_controller" && c.state == "running") {
          running = true;
        }
      }
    }
  }

  // fill ROS message
  geometry_msgs::Twist tw;

  tw.linear.x = 1.0;

  // publish ROS message
  pub.publish(tw);

  ros::spinOnce();

  return 0;
}
