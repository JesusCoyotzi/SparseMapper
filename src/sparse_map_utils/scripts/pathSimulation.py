#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from sparse_map_msgs.srv import MakePlan
from std_msgs.msg import Header
import sys
import random


def callPlanners(occ, free, experiments):
    moveServer = rospy.ServiceProxy("move_make_plan", GetPlan)
    sparseServer = rospy.ServiceProxy("sparse_make_plan", MakePlan)

    # try:
    #     moveServer.wait_for_service('sparse_make_plan', 1.0)
    #     sparseServer.wait_for_service('move_make_plan', 1.0)
    # except rospy.exceptions.ROSException, e:
    #     print("Service call failed %s" % e)

    starts = random.sample(free, experiments)
    goals = random.sample(free, experiments)
    head = Header()
    head.stamp = rospy.Time.now()
    head.frame_id = "map"
    for start, goal in zip(starts, goals):
        moveStart = PoseStamped(header=head, pose=start)
        moveGoal = PoseStamped(header=head, pose=goal)

        sparseStart = start.position
        sparseGoal = goal.position

        try:
            moveResponse = moveServer(moveStart, moveGoal, 0.3)
            print(moveResponse.plan)
        except rospy.ServiceException, e:
            print("Service call failed %s" % e)

        try:
            sparseResponse = sparseServer(sparseStart, sparseGoal)
            print(sparseResponse.path)
        except rospy.ServiceException, e:
            print("Service call failed %s" % e)

    return


def main():
    if len(sys.argv) < 2:
        print("Usage: program experiments")
    experiments = int(sys.argv[1])
    rospy.init_node("pathSimulator", anonymous=False)
    try:
        rospy.wait_for_service('static_map', 5.0)
        mapSrv = rospy.ServiceProxy("static_map", GetMap)
        response = mapSrv()
        grid = response.map
    except rospy.ServiceException, e:
        print("Service call failed %s" % e)

    free_grids = []
    occ_grids = []

    width = grid.info.width
    height = grid.info.height
    resolution = grid.info.resolution
    origin = grid.info.origin.position
    print("Got a map of:[{},{}] @ ".format(width, height, resolution))
    print("Origin in: %s" % origin)
    cadena = "Free Space at [{},{}] = [{},{}]"
    for i in range(width):
        for j in range(height):
            if grid.data[i + width * j] == 0:
                x = origin.x + i * resolution
                y = origin.y + j * resolution
                p = Pose(Point(x, y, 0), Quaternion(0, 0, 0, 1))
                free_grids.append(p)
            elif grid.data[i + width * j] > 0:
                x = origin.x + i * resolution
                y = origin.y + j * resolution
                p = Pose(Point(x, y, 0), Quaternion(0, 0, 0, 1))
                occ_grids.append(p)

    print("Got %s occupied cells" % len(occ_grids))
    print("Got %s free cells" % len(free_grids))
    #print(cadena.format(i, j, x, y))

    callPlanners(free_grids, occ_grids, experiments)

    # for i, p in enumerate(grid.data):
    #     if p > 80 and p > 0:
    #         print("Occupied cell! %s" % p)


if __name__ == '__main__':
    main()
