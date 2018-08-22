#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
import actionlib
import control_msgs.msg
import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg

rospy.init_node('test')

# initialize action client
cli = actionlib.SimpleActionClient(
    '/hsrb/arm_trajectory_controller/follow_joint_trajectory',
    control_msgs.msg.FollowJointTrajectoryAction)

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
        if c.name == 'arm_trajectory_controller' and c.state == 'running':
            running = True

# fill and send task1
goal = control_msgs.msg.FollowJointTrajectoryGoal()
traj = trajectory_msgs.msg.JointTrajectory()
traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
                    "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
p = trajectory_msgs.msg.JointTrajectoryPoint()
p.positions = [0.2, -0.5, 0, 0, 0]
p.velocities = [0, 0, 0, 0, 0]
p.time_from_start = rospy.Time(3)
traj.points = [p]
goal.trajectory = traj
cli.send_goal(goal)

# wait for a while to do task1
rospy.sleep(1)

# fill and send task2 (overwrite task1)
p = trajectory_msgs.msg.JointTrajectoryPoint()
p.positions = [0, 0, 0, 0, 0]
p.velocities = [0, 0, 0, 0, 0]
p.time_from_start = rospy.Time(3)
traj.points = [p]
goal.trajectory = traj
cli.send_goal(goal)

# wait for task2 to finish
cli.wait_for_result()
