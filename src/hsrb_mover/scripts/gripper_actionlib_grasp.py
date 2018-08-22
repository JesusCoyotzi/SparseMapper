#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
import actionlib
import controller_manager_msgs.srv
import rospy
import tmc_control_msgs.msg

rospy.init_node('test')

# initialize action client
cli = actionlib.SimpleActionClient(
    '/hsrb/gripper_controller/grasp',
    tmc_control_msgs.msg.GripperApplyEffortAction)

# wait for the action server to establish connection
cli.wait_for_server()

# make sure the controller is running
rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
list_controllers = rospy.ServiceProxy(
    '/hsrb/controller_manager/list_controllers',
    controller_manager_msgs.srv.ListControllers)
running = False
while running is False:
    rospy.sleep(0.1)
    for c in list_controllers().controller:
        if c.name == 'gripper_controller' and c.state == 'running':
            running = True

# fill ROS message
goal = tmc_control_msgs.msg.GripperApplyEffortGoal()
goal.effort = -0.5

# send message to the action server
cli.send_goal(goal)

# wait for the action server to complete the order
cli.wait_for_result()
