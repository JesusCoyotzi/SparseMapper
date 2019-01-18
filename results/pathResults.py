import sys
import os

import pandas as pd

import matplotlib.pyplot as plt

def main():
    if len(sys.argv) < 2:
        print("Error usage:")
        print("pathResults.py file to process")
        return
    paths=pd.read_csv(sys.argv[1])
    valid_nav = paths["nav_tray"].notna()
    valid_sparse = paths["sparse_tray"].notna()
    #print(paths["direct_distance"].loc[valid_nav])
    nav_trays = paths["nav_tray"].loc[valid_nav]
    nav_times = paths["nav_time"].loc[valid_nav]
    sparse_trays = paths["sparse_tray"].loc[valid_sparse]
    sparse_times = paths["sparse_time"].loc[valid_sparse]

    print(nav_trays.mean(),nav_trays.std())
    print(sparse_trays.mean(),sparse_trays.std())

    plt.style.use("seaborn-whitegrid")
    plt.figure(1)
    plt.scatter(paths["direct_distance"].loc[valid_nav],nav_trays,s=1)
    plt.scatter(paths["direct_distance"].loc[valid_sparse],sparse_trays,s=1)
    #plt.plot(paths["direct_distance"],paths["sparse_tray"])
    plt.figure(2)
    plt.scatter(paths["direct_distance"].loc[valid_nav],nav_times,s=1)
    plt.scatter(paths["direct_distance"].loc[valid_sparse],sparse_times,s=1)
    plt.figure(3)
    plt.scatter(paths["direct_distance"].loc[valid_nav],nav_times,s=1)
    plt.scatter(paths["direct_distance"].loc[valid_sparse],sparse_times,s=1)

    plt.show()

if __name__ == '__main__':
    main()
