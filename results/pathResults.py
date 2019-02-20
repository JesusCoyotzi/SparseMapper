import sys
import os

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import json
from scipy.signal import savgol_filter
from collections import defaultdict


def formatGrph(x, y_nav, y_spr, filter=False, samples=5):
    sz = (10, 6)
    f = plt.figure(figsize=sz)
    if filter:
        y_hat_nav = savgol_filter(y_nav, smpls, 3)
        y_hat_spr = savgol_filter(y_spr, smpls, 3)
    else:
        y_hat_nav = y_nav
        y_hat_spr = y_spr

    plt.plot(x, y_hat_nav, linestyle='-',
             linewidth=1, marker="", markersize=2, label="Navigation")
    plt.plot(x, y_hat_spr, linestyle='--',
             linewidth=1, marker="", markersize=2, label="Sparse")

    return f


def main():
    if len(sys.argv) < 2:
        print("Error usage:")
        print("pathResults.py file to process, [outputFolder]")
        return

    filename = sys.argv[1]
    paths = pd.read_csv(sys.argv[1])
    nav_fails = paths["nav_tray"].isna().sum()
    sparse_fails = paths["sparse_tray"].isna().sum()
    n_paths = len(paths.index)
    nav_error_ratio = nav_fails / n_paths
    sparse_error_ratio = sparse_fails / n_paths

    paths.dropna(inplace=True)
    paths.sort_values("direct_distance", inplace=True)
    valid_nav = paths["nav_tray"]
    valid_sparse = paths["sparse_tray"]
    valid_paths = len(paths.index)
    # print(paths["direct_distance"].loc[valid_nav])

    nav_trays = paths["nav_tray"]
    nav_times = paths["nav_time"]
    nav_jerk = paths["nav_jerk"]
    nav_nodes = paths["nav_nodes"]
    nav_twist = paths["nav_twist"]
    nav_std = paths["nav_std"]

    sparse_trays = paths["sparse_tray"]
    sparse_times = paths["sparse_time"]
    sparse_jerk = paths["sparse_jerk"]
    sparse_nodes = paths["sparse_nodes"]
    sparse_twist = paths["sparse_twist"]
    sparse_std = paths["sparse_std"]

    msg_tray = "Navigation {} | Sparse: {}"
    print("File processed:" + filename)
    print("Has {} paths {} are valid".format(n_paths, valid_paths))
    print("Path Errors")
    print(msg_tray.format(nav_fails, sparse_fails))
    print(msg_tray.format(nav_error_ratio, sparse_error_ratio))
    print("Trayectory averages")
    print(msg_tray.format(nav_trays.mean(), sparse_trays.mean()))
    print("Time averages")
    print(msg_tray.format(nav_times.mean(), sparse_times.mean()))
    print("Nodes averages")
    print(msg_tray.format(nav_nodes.mean(), sparse_nodes.mean()))
    print("Smoothness averages")
    print(msg_tray.format(nav_twist.mean(), sparse_twist.mean()))
    print("Std Avg averages")
    print(msg_tray.format(nav_std.mean(), sparse_std.mean()))
    print("*" * 15)

    plt.style.use("seaborn-whitegrid")
    plt.rcParams.update({'font.size': 14})
    x = paths["direct_distance"]

    f1 = formatGrph(x, nav_trays, sparse_trays)
    plt.xlabel("Distancias entre inicio y destino [m]")
    plt.ylabel("Longitud de la trayectoria [m]")
    plt.title("Comparación de longitud")
    plt.grid(True)
    plt.legend()

    f2 = formatGrph(x, nav_times, sparse_times)
    plt.xlabel("Distancias entre inicio y destino [m]")
    plt.ylabel("Tiempo de ejecución [s]")
    plt.title("Velocidad de ejecución")
    plt.grid(True)
    plt.legend()

    f3 = formatGrph(x, nav_nodes, sparse_nodes)
    plt.xlabel("Distancias entre inicio y destino [m]")
    plt.ylabel("Nodos")
    plt.title("Numero de nodos en trayectoria")
    plt.grid(True)
    plt.legend()

    f4 = formatGrph(x, nav_twist, sparse_twist)
    plt.xlabel("Distancias entre inicio y destino [m]")
    plt.ylabel("Angulo acumulado [r]")
    plt.title("Tortuosidad")
    plt.grid(True)
    plt.legend()

    f5 = formatGrph(x, nav_std, sparse_std)
    plt.xlabel("Distancias entre inicio y destino [m]")
    plt.ylabel("Desviación Estandar [m]")
    plt.title("Dispersión de las trayectorias")
    plt.grid(True)
    plt.legend()

    # plt.show()

    # basename = sys.argv[1].split('.')[0]

    if len(sys.argv) < 3:
        # outputFolder = os.path.dirname(sys.argv[1])
        plt.show()
    else:
        payload = defaultdict(dict)
        micro = {}
        outputFolder = sys.argv[2]
        basename = os.path.basename(sys.argv[1]).split('.')[0]

        f1.savefig(outputFolder + "longitud" + basename + ".png")
        f2.savefig(outputFolder + "tiempo" + basename + ".png")
        f3.savefig(outputFolder + "nodos" + basename + ".png")
        f4.savefig(outputFolder + "tortuosidad" + basename + ".png")
        f5.savefig(outputFolder + "dispersion" + basename + ".png")

        payload["filename"] = basename
        payload["n_paths"] = n_paths
        payload["valid_paths"] = valid_paths
        payload["nav_errors"] = nav_fails.item()
        payload["sparse_errors"] = sparse_fails.item()
        payload["nav_error_ratio"] = nav_error_ratio
        payload["sparse_error_ratio"] = sparse_error_ratio
        micro["nav_mean"] = nav_nodes.mean()
        micro["nav_std"] = nav_nodes.std()
        micro["sparse_mean"] = sparse_nodes.mean()
        micro["sparse_std"] = sparse_nodes.std()
        payload["nodes"] = dict(micro)
        micro["nav_mean"] = nav_trays.mean()
        micro["nav_std"] = nav_trays.std()
        micro["sparse_mean"] = sparse_trays.mean()
        micro["sparse_std"] = sparse_trays.std()
        payload["length"] = dict(micro)
        micro["nav_mean"] = nav_times.mean()
        micro["nav_std"] = nav_times.std()
        micro["sparse_mean"] = sparse_times.mean()
        micro["sparse_std"] = sparse_times.std()
        payload["runtime"] = dict(micro)
        micro["nav_mean"] = nav_twist.mean()
        micro["nav_std"] = nav_twist.std()
        micro["sparse_mean"] = sparse_twist.mean()
        micro["sparse_std"] = sparse_twist.std()
        payload["twist"] = dict(micro)
        micro["nav_mean"] = nav_std.mean()
        micro["nav_std"] = nav_std.std()
        micro["sparse_mean"] = sparse_std.mean()
        micro["sparse_std"] = sparse_std.std()
        payload["std"] = dict(micro)
        # print(payload)

        with open(outputFolder + basename + "estadisticos.json", 'w') as statsFile:
            json.dump(payload, statsFile, sort_keys=True, indent=4)

    # f3.savefig(bs_name + "traj.eps")
    # f3.savefig(bs_name + "time.eps")
    # f3.savefig(bs_name + "smooth.eps")


if __name__ == '__main__':
    main()
