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
import csv
from math import sqrt


file_header = ["orig_x", "orig_y", "orig_z", "goal_x", "goal_y", "goal_z",
               "direct_distance", "reachable",
               "nav_tray", "nav_nodes", "nav_time",
               "sparse_tray", "sparse_nodes", "sparse_time"]


def calcTrajectoryDist(path):
    # receives a path
    total_dist = 0.0
    n_poses = len(path.poses)
    for i in range(1, n_poses):
        total_dist += euclideanDistance(
            path.poses[i].pose, path.poses[i - 1].pose)
    return total_dist


def euclideanDistance(p1, p2):
    # Gets two pose objects
    x_o = p1.position.x
    y_o = p1.position.y
    z_o = p1.position.z
    x_d = p2.position.x
    y_d = p2.position.y
    z_d = p2.position.z
    return sqrt((x_d - x_o)**2 + (y_d - y_o)**2 + (z_d - z_o)**2)


def setupResults(start, goal, distance,reachable=True):
    results_dic = dict.fromkeys(file_header, float("nan"))
    results_dic["orig_x"] = start.position.x
    results_dic["orig_y"] = start.position.y
    results_dic["orig_z"] = start.position.z

    results_dic["goal_x"] = goal.position.x
    results_dic["goal_y"] = goal.position.y
    results_dic["goal_z"] = goal.position.z
    results_dic["direct_distance"] = distance
    results_dic["reachable"] =reachable

    return results_dic


def callPlanners(occ, free, experiments, output_file_name="results.csv"):
    moveServer = rospy.ServiceProxy("move_make_plan", GetPlan)
    sparseServer = rospy.ServiceProxy("sparse_make_plan", MakePlan)

    try:
        moveServer.wait_for_service(1.0)
        sparseServer.wait_for_service(1.0)
    except rospy.exceptions.ROSException, e:
        print("Service call failed %s" % e)

    print("Writing results to %s" % output_file_name)

    with open(output_file_name, 'w') as f:
        csv_writer = csv.DictWriter(f, fieldnames=file_header)
        csv_writer.writeheader()

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

        moveResponse = None
        sparseResponse = None

        straight_distance = euclideanDistance(start, goal)
        simulation_result = setupResults(start, goal, straight_distance)

        try:
            moveTime = rospy.Time.now()
            moveResponse = moveServer(moveStart, moveGoal, 0.3)
            nav_time = moveTime - rospy.Time.now()
            nav_pth = moveResponse.plan
            if nav_pth.poses:
                nav_stck_nodes = len(nav_pth.poses)
                nav_tray_dist = calcTrajectoryDist(nav_pth)
                simulation_result["nav_nodes"] = nav_stck_nodes
                simulation_result["nav_tray"] = nav_tray_dist
                simulation_result["nav_time"] = nav_time
        except rospy.ServiceException, e:
            print("Service call failed %s" % e)

        try:
            sparseTime = rospy.Time.now()
            sparseResponse = sparseServer(sparseStart, sparseGoal)
            sparseTime = sparseTime - rospy.Time.now()
            sparse_pth = sparseResponse.plan
            if sparse_pth.poses:
                spr_stck_nodes = len(sparse_pth.poses)
                spr_traj_dist = calcTrajectoryDist(sparse_pth)
                simulation_result["sparse_nodes"] = spr_stck_nodes
                simulation_result["sparse_tray"] = spr_traj_dist
                simulation_result["sparse_time"] = sparseTime
        except rospy.ServiceException, e:
            print("Service call failed %s" % e)

        with open(output_file_name, 'a') as f:
            csv_writer = csv.DictWriter(f, fieldnames=file_header)
            csv_writer.writerow(simulation_result)
            # nav_stck_execution =

    return


def main():
    if len(sys.argv) < 3:
        print("Usage: program experiments resultfile_name")
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

    callPlanners(free_grids, occ_grids, experiments, sys.argv[2])

    # for i, p in enumerate(grid.data):
    #     if p > 80 and p > 0:
    #         print("Occupied cell! %s" % p)


if __name__ == '__main__':
    main()