#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan

start = PoseStamped()

def goalCallback(msg):
    goal = msg
    try:
        goalProxy = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
        pth = goalProxy(start, goal,0.3)
        print("NavStack path planing succesfull")
        for p in pth.plan.poses:
            print((p.pose.position.x,p.pose.position.y,p.pose.position.z))
    except rospy.ServiceException as e:
        print("Service call failed %s"%e)

def startCallBack(msg):
    global start
    start = PoseStamped(msg.header,msg.pose.pose)
    #print(msg)

def setup():
    rospy.init_node("goal_emitter", anonymous=False)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goalCallback)
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, startCallBack)
    rospy.wait_for_service("/move_base/make_plan")
    rospy.spin()


if __name__ == '__main__':
    setup()
