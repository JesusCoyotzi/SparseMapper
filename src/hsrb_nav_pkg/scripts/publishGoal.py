#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan


def goalCallback(msg):
    hd = Header(stamp=msg.header.stamp,frame_id=msg.header.frame_id)
    ps_start = Pose(Point(0,0,0),Quaternion(0,0,0,1))
    goal_pose = Pose(msg.point,Quaternion(0,0,0,1))
    start = PoseStamped(hd,ps_start)
    goal = PoseStamped(hd,goal_pose)

    try:
        goalProxy = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
        pth = goalProxy(start, goal,0.5)
        print("Path planing succesfull")
    except rospy.ServiceException as e:
        print("Service call failed %s" % e)


def setup():
    rospy.init_node("goal_emitter", anonymous=False)
    rospy.Subscriber("/clicked_point", PointStamped, goalCallback)
    rospy.wait_for_service("/move_base/make_plan")
    rospy.spin()


if __name__ == '__main__':
    setup()
