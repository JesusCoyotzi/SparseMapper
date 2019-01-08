#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from std_msgs.msg import Header
from sparse_map_msgs.srv import MakePlan


def goalCallback(msg):
    start = Point(0, 0, 0)
    goal = Point(msg.point.x, msg.point.y, msg.point.z)
    head = Header()
    head.stamp = rospy.Time.now()
    head.frame_id = "map"
    unitQ = Quaternion(0,0,0,1)
    print("Got a goal:")
    print(goal)
    try:
        goalProxy = rospy.ServiceProxy("sparse_make_plan", MakePlan)
        pth = goalProxy(start, goal)
        print("SparMap->path planing succesfull")
    except rospy.ServiceException as e:
        print("Service call failed %s" % e)

    try:
        goalProxy = rospy.ServiceProxy("move_make_plan", GetPlan)

        moveStrt = PoseStamped(head,Pose(start,unitQ))
        moveGoal = PoseStamped(head,Pose(goal,unitQ))
        pth = goalProxy(moveStrt, moveGoal,0.3)
        print("MoveBase->path planing succesfull")
    except rospy.ServiceException as e:
        print("Service call failed %s" % e)


def setup():
    rospy.init_node("goal_emitter", anonymous=False)
    print("Starting micro server for move base and sparse map")
    rospy.Subscriber("/clicked_point", PointStamped, goalCallback)
    rospy.wait_for_service("sparse_make_plan")
    rospy.wait_for_service("move_make_plan")
    rospy.spin()


if __name__ == '__main__':
    setup()
