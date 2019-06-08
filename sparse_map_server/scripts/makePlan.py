#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from sparse_map_msgs.srv import MakePlan

start = Point(3, -4, 0)

def goalCallback(msg):
    goal = Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
    try:
        goalProxy = rospy.ServiceProxy("/sparse_map/make_plan", MakePlan)
        pth = goalProxy(start, goal)
        print("Path planing succesfull:")
        for p in pth.plan.poses:
            print((p.pose.position.x,p.pose.position.y,p.pose.position.z))
    except rospy.ServiceException as e:
        print("Service call failed %s"%e)

def startCallBack(msg):
    global start
    start = Point(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
    #print(msg)

def setup():
    rospy.init_node("goal_emitter", anonymous=False)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goalCallback)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, startCallBack)
    rospy.wait_for_service("/sparse_map/make_plan")
    rospy.spin()


if __name__ == '__main__':
    setup()
