/// @brief Copyright (C) 2016 Toyota Motor Corporation
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <ros/ros.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");

  // initialize action client
  Client cli("/hsrb/omni_base_controller/follow_joint_trajectory", true);

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
        if (c.name == "omni_base_controller" && c.state == "running") {
          running = true;
        }
      }
    }
  }

  // fill ROS message
  control_msgs::FollowJointTrajectoryGoal goal;

  goal.trajectory.joint_names.push_back("odom_x");
  goal.trajectory.joint_names.push_back("odom_y");
  goal.trajectory.joint_names.push_back("odom_t");

  goal.trajectory.points.resize(1);

  goal.trajectory.points[0].positions.resize(3);
  goal.trajectory.points[0].positions[0] = 1.0;
  goal.trajectory.points[0].positions[1] = 1.0;
  goal.trajectory.points[0].positions[2] = 0.0;
  goal.trajectory.points[0].velocities.resize(3);
  for (size_t i = 0; i < 3; ++i) {
    goal.trajectory.points[0].velocities[i] = 0.0;
  }
  goal.trajectory.points[0].time_from_start = ros::Duration(6.0);

  // send message to the action server
  cli.sendGoal(goal);

  // wait for the action server to complete the order
  cli.waitForResult(ros::Duration(10.0));

  return 0;
}
