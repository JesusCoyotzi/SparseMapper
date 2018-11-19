#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from sparse_map_msgs.srv import MakePlan


def goalCallback(msg):
    start = Point(0, 0, 0)
    goal = Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
    try:
        goalProxy = rospy.ServiceProxy("/make_plan", MakePlan)
        pth = goalProxy(start, goal)
        print("pat planing succesfull")
    except rospy.ServiceException as e:
        print("Service call failed %s"%e)

def setup():
    rospy.init_node("goal_emitter", anonymous=False)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goalCallback)
    rospy.wait_for_service("/make_plan")
    rospy.spin()


if __name__ == '__main__':
    setup()
