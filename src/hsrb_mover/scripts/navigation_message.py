#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
from geometry_msgs.msg import Point, PoseStamped, Quaternion
import rospy
import tf.transformations

rospy.init_node('test')

# initialize ROS publisher
pub = rospy.Publisher('goal', PoseStamped, queue_size=10)

# wait to establish connection between the navigation interface
while pub.get_num_connections() == 0:
    rospy.sleep(0.1)

# input goal pose
goal_x = 1
goal_y = 0
goal_yaw = 0

# fill ROS message
goal = PoseStamped()
goal.header.stamp = rospy.Time.now()
goal.header.frame_id = "map"
goal.pose.position = Point(goal_x, goal_y, 0)
quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
goal.pose.orientation = Quaternion(*quat)

# publish ROS message
pub.publish(goal)
