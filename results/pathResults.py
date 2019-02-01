import sys
import os

import pandas as pd

import matplotlib.pyplot as plt

import numpy as np

def main():
    if len(sys.argv) < 2:
        print("Error usage:")
        print("pathResults.py file to process")
        return
    bs_name= sys.argv[1].split('.')[0]

    paths=pd.read_csv(sys.argv[1])
    paths.dropna(inplace=True)
    paths.sort_values("direct_distance",inplace=True)
    valid_nav = paths["nav_tray"]
    valid_sparse = paths["sparse_tray"]
    #print(paths["direct_distance"].loc[valid_nav])

    nav_trays = paths["nav_tray"]
    nav_times = paths["nav_time"]
    nav_jerk = paths["nav_jerk"]
    nav_nodes = paths["nav_nodes"]
    nav_twist = paths["nav_twist"]
    sparse_trays = paths["sparse_tray"]
    sparse_times = paths["sparse_time"]
    sparse_jerk = paths["sparse_jerk"]
    sparse_nodes = paths["sparse_nodes"]
    sparse_twist = paths["sparse_twist"]


    msg_tray = "Navigation: {} | Sparse{} "
    print("Trayectory averages")
    print(msg_tray.format(nav_trays.mean(),sparse_trays.mean()))
    print("Time averages")
    print(msg_tray.format(nav_times.mean(),sparse_times.mean()))
    print("Smoothness averages")
    print(msg_tray.format(nav_twist.mean(),sparse_twist.mean()))

    barWidth = 0.5


    #fig, ax1 = plt.subplots()


    sz = (10,6)
    x = np.arange(0,len(nav_trays))
    f1=plt.style.use("seaborn-whitegrid")
    plt.figure(1,figsize=sz)
    plt.plot(paths["direct_distance"],nav_trays,label="navigation")
    plt.plot(paths["direct_distance"],sparse_trays,label="sparse")
    plt.xlabel("Straight Distance [m]")
    plt.ylabel("Trajectory lenght [m]")
    plt.title("lenght")
    plt.legend()


    f2=plt.figure(2,figsize=sz)
    plt.plot(paths["direct_distance"],nav_times,label="navigation")
    plt.plot(paths["direct_distance"],sparse_times,label="sparse")
    plt.xlabel("Straight Distance [m]")
    plt.ylabel("Exection Time [s]")
    plt.title("Runtime")
    plt.legend()

    f3=plt.figure(3,figsize=sz)
    plt.plot(paths["direct_distance"],nav_nodes,label="navigation")
    plt.plot(paths["direct_distance"],sparse_nodes,label="sparse")
    plt.xlabel("Straight Distance [m]")
    plt.ylabel("Number of nodes")
    plt.title("Nodes in trajectory")
    plt.legend()

    f4=plt.figure(4,figsize=sz)
    plt.plot(paths["direct_distance"],nav_twist,label="navigation")
    plt.plot(paths["direct_distance"],sparse_twist,label="sparse")
    plt.xlabel("Distancia entre puntos [m]")
    plt.ylabel("Angulo [rad]")
    plt.title("Tortuosidad")
    plt.legend()

    # f3=plt.figure(3,figsize=sz)
    # plt.scatter(paths["direct_distance"],nav_jerk,label="navigation")
    # plt.scatter(paths["direct_distance"],sparse_jerk,label="sparse")
    # plt.xlabel("Straight Distance [m]")
    # plt.ylabel("Third derivate average")
    # plt.title("Trajectory Smoothness")
    # plt.legend()
    plt.show()

    # f3.savefig(bs_name + "traj.eps")
    # f3.savefig(bs_name + "time.eps")
    # f3.savefig(bs_name + "smooth.eps")


if __name__ == '__main__':
    main()
