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

from math import sqrt
import sys
import random
import csv
import numpy as np
import pandas as pd


np.set_printoptions(suppress=True,precision=3)

file_header = ["orig_x", "orig_y", "goal_x", "goal_y",
               "direct_distance", "reachable",
               "nav_tray", "nav_twist", "nav_time",
               "nav_jerk", "nav_std", "nav_nodes",
               "sparse_tray", "sparse_twist", "sparse_time",
               "sparse_jerk", "sparse_std", "sparse_nodes"
               ]


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


def getJerk(path):
    key_points = [[p.pose.position.x, p.pose.position.y,
                   p.pose.position.z] for p in path.poses]
    points_np = np.array(key_points)
    # print(points_np[0:5])
    frst_dev = np.gradient(points_np)
    scnd_dev = np.gradient(frst_dev)
    thrd_dev = np.gradient(scnd_dev)
    jerk = np.linalg.norm(thrd_dev, axis=1)
    jerk_mean = np.mean(jerk)
    return jerk_mean


def calcAngle(v1, v2):
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)

    v1_u = v1 / n1
    v2_u = v2 / n2

    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def getTwist(path):
    key_points = [[p.pose.position.x, p.pose.position.y,
                   p.pose.position.z] for p in path.poses]
    points_np = np.array(key_points)
    n_poses = len(path.poses)
    twist = 0.0
    for i in range(1, n_poses - 1):
        vi = points_np[i] - points_np[i - 1]
        vj = points_np[i + 1] - points_np[i]
        # print("***")
        # print(points_np[i - 1])
        # print(points_np[i])
        # print(points_np[i + 1])
        # print(vi)
        # print(vj)
        #print("***")

        if vi.any() and vj.any():
            twist = twist + np.abs(calcAngle(vi, vj))
        #print(twist)
#

#        print((points_np[i+1],points_np[i],points_np[i-1]))
    return twist


def getStdDev(pth):
    # Get the std deviation of the
    key_points = [[p.pose.position.x, p.pose.position.y,
                   p.pose.position.z] for p in pth.poses]
    points_np = np.array(key_points)
    m = np.mean(points_np, axis=0)
    return np.sqrt(np.mean(np.abs(points_np - m)**2))


def setupResults(start, goal, distance, reachable=True):
    # z is irrelevant in this case but not generally
    results_dic = dict.fromkeys(file_header, float("nan"))
    results_dic["orig_x"] = start.position.x
    results_dic["orig_y"] = start.position.y

    results_dic["goal_x"] = goal.position.x
    results_dic["goal_y"] = goal.position.y

    results_dic["direct_distance"] = distance
    results_dic["reachable"] = reachable

    # results_list = [start.position.x, start.position.y,
    #                 goal.position.x, goal.position.y,
    #                 distance, reachable]

    return results_dic


def processPath(head, start, goal,
                reachable,
                sparseServer, moveServer):
    moveStart = PoseStamped(header=head, pose=start)
    moveGoal = PoseStamped(header=head, pose=goal)

    sparseStart = start.position
    sparseGoal = goal.position

    moveResponse = None
    sparseResponse = None

    straight_distance = euclideanDistance(start, goal)
    simulation_result = setupResults(start, goal, straight_distance, reachable)
    try:
        sparseTime = rospy.Time.now()
        sparseResponse = sparseServer(sparseStart, sparseGoal)
        sparseTime = rospy.Time.now() - sparseTime
        sparse_pth = sparseResponse.plan
        if sparse_pth.poses:
            spr_stck_nodes = len(sparse_pth.poses)
            spr_traj_dist = calcTrajectoryDist(sparse_pth)
            spr_jrk = getJerk(sparse_pth)
            spr_std = getStdDev(sparse_pth)
            spr_twst = getTwist(sparse_pth)
            simulation_result["sparse_nodes"] = spr_stck_nodes
            simulation_result["sparse_tray"] = spr_traj_dist
            simulation_result["sparse_time"] = sparseTime.to_sec()
            simulation_result["sparse_jerk"] = spr_jrk
            simulation_result["sparse_std"] = spr_std
            simulation_result["sparse_twist"] = spr_twst
            # simulation_result.extend(
            #     [spr_stck_nodes, spr_traj_dist,
            #      sparseTime.to_sec(), spr_jrk])

    except rospy.ServiceException, e:
        print("Service call failed %s" % e)

    try:
        moveTime = rospy.Time.now()
        moveResponse = moveServer(moveStart, moveGoal, 0.125)
        nav_time = rospy.Time.now() - moveTime
        nav_pth = moveResponse.plan
        if nav_pth.poses:
            nav_stck_nodes = len(nav_pth.poses)
            nav_tray_dist = calcTrajectoryDist(nav_pth)
            nav_jrk = getJerk(nav_pth)
            nav_std = getStdDev(nav_pth)
            nav_twst = getTwist(nav_pth)
            simulation_result["nav_nodes"] = nav_stck_nodes
            simulation_result["nav_tray"] = nav_tray_dist
            simulation_result["nav_time"] = nav_time.to_sec()
            simulation_result["nav_jerk"] = nav_jrk
            simulation_result["nav_std"] = nav_std
            simulation_result["nav_twist"] = nav_twst
            # simulation_result.extend(
            #     [nav_stck_nodes, nav_tray_dist,
            #      nav_time.to_sec(), nav_jrk])

    except rospy.ServiceException, e:
        print("Service call failed %s" % e)

    return simulation_result


def callPlanners(occ, free, experiments, output_file_name="results.csv", invalid=False):
    moveServer = rospy.ServiceProxy("move_make_plan", GetPlan)
    sparseServer = rospy.ServiceProxy("sparse_make_plan", MakePlan)

    try:
        moveServer.wait_for_service()
        sparseServer.wait_for_service()
    except rospy.exceptions.ROSException, e:
        print("Service call failed %s" % e)

    print("Writing results to %s" % output_file_name)

    # with open(output_file_name, 'w') as f:
    #     csv_writer = csv.DictWriter(f, fieldnames=file_header,quoting=csv.QUOTE_NONE)
    #     csv_writer.writeheader()

    results_df = pd.DataFrame(columns=file_header)

    if invalid:
        starts_valid = random.sample(free, experiments / 2)
        goals_valid = random.sample(free, experiments / 2)
        starts_blocked = random.sample(occ, experiments / 2)
        goals_blocked = random.sample(occ, experiments / 2)
    else:
        starts_valid = random.sample(free, experiments)
        goals_valid = random.sample(free, experiments)

    head = Header()
    head.stamp = rospy.Time.now()
    head.frame_id = "map"
    full_results = []
    for start, goal in zip(starts_valid, goals_valid):
        simul_res = processPath(head, start, goal, True,
                                sparseServer, moveServer)

        full_results.append(simul_res)
        # with open(output_file_name, 'a') as f:
        #     csv_writer = csv.DictWriter(f, fieldnames=file_header,quoting=csv.QUOTE_NONE)
        #     csv_writer.writerow(simul_res)

    if invalid:
        for start, goal in zip(starts_blocked, goals_blocked):
            simul_res = processPath(head, start, goal, False,
                                    sparseServer, moveServer)
            full_results.append(simul_res)
            # with open(output_file_name, 'a') as f:
            #     csv_writer = csv.DictWriter(f, fieldnames=file_header,quoting=csv.QUOTE_NONE)
            #     csv_writer.writerow(simul_res)

    # print(simul_res)
    results_df = results_df.append(full_results)
    results_df.to_csv(output_file_name, float_format='%.5f')

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

    callPlanners(occ_grids, free_grids, experiments, sys.argv[2])

    # for i, p in enumerate(grid.data):
    #     if p > 80 and p > 0:
    #         print("Occupied cell! %s" % p)


if __name__ == '__main__':
    main()
