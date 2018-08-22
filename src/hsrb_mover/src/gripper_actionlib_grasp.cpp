/// @brief Copyright (C) 2016 Toyota Motor Corporation
#include <actionlib/client/simple_action_client.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <ros/ros.h>
#include <tmc_control_msgs/GripperApplyEffortAction.h>
#include <tmc_control_msgs/GripperApplyEffortGoal.h>

typedef actionlib::SimpleActionClient<tmc_control_msgs::GripperApplyEffortAction> Client;

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");

  // initialize action client
  Client cli("/hsrb/gripper_controller/grasp", true);

  // wait for the action server to establish connection
  cli.waitForServer();

  // make sure the controller is running
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<controller_manager_msgs::ListControllers>(
      "/hsrb/controller_manager/list_controllers");
  controller_manager_msgs::ListControllers list_controllers;
  bool running = false;
  while (running == false) {
    ros::Duration(0.1).sleep();
    if (client.call(list_controllers)) {
      for (unsigned int i = 0; i < list_controllers.response.controller.size(); i++) {
        controller_manager_msgs::ControllerState c = list_controllers.response.controller[i];
        if (c.name == "gripper_controller" && c.state == "running") {
          running = true;
        }
      }
    }
  }

  // fill ROS message
  tmc_control_msgs::GripperApplyEffortGoal goal;

  goal.effort = -0.5;

  // send message to the action server
  cli.sendGoal(goal);

  // wait for the action server to complete the order
  cli.waitForResult(ros::Duration(5.0));

  return 0;
}
