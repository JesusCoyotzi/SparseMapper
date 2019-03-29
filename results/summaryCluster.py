import glob
import sys
import os
import pandas as pd
import numpy as np


def main():
    if len(sys.argv) < 2:
        print("Error no folder specified?")
        return

    folder = os.path.abspath(sys.argv[1])
    globber = folder + "/*.csv"
    csvs = glob.glob(globber)
    cluster_tupl = (16, 32, 64, 128, 256, 512)
    mtd_tpl = ("kmeansCPU", "inner", "kpp", "LBG")

    print("Using folder:{}".format(folder))
    list_of_df = []
    for file in csvs:
        print("For file %s" % (file))
        data = pd.read_csv(file, index_col=0)
        list_of_df.append(data)

    overall_df = pd.concat(list_of_df)
    summarized_df = pd.DataFrame(
        columns=["dataset", "clusters", "method", "distorsion", "time"])
    # print(overall_df)
    list_of_dict = []
    for mtd in mtd_tpl:
        for clt in cluster_tupl:
            local_dict = {"dataset": os.path.basename(folder)}
            mask = (overall_df["clusters"] == clt) & (
                overall_df["method"] == mtd)
            pruned_df = overall_df[mask]
            local_dict["method"] = mtd
            local_dict["clusters"] = clt
            local_dict["distorsion"] = pruned_df["distorsion"].mean()
            local_dict["time"] = pruned_df["time"].mean()
            list_of_dict.append(local_dict)

    summary_df = pd.DataFrame(list_of_dict)
    # summary_df.to_csv("summary"+os.path.basename(folder)+".csv")
    print(summary_df)

    # ltx_ln = " {clusters} & {kmeansCPU:.4f}  & {inner:.4f}  & {LBG:.4f}  & {kpp:.4f} \\\\"
    # info_dist = []
    # info_time = []
    # for clt in cluster_tupl:
    #     clt_msk = summary_df["clusters"] == clt
    #     nodes_df = summary_df[clt_msk]
    #     dic_dist = {"clusters": clt}
    #     dic_time = {"clusters": clt}
    #     for method in mtd_tpl:
    #         method_msk = nodes_df["method"] == method
    #         mtd_df = nodes_df[method_msk]
    #         dic_dist[method] = mtd_df["distorsion"].iloc[0]
    #         dic_time[method] = mtd_df["time"].iloc[0]
    #     info_dist.append(dic_dist)
    #     info_time.append(dic_time)
    #
    # print("Distorsion" * 5)
    # print(" clusters \t kmeansCPU \t inner \t LBG \t kpp")
    # for d in info_dist:
    #     print(ltx_ln.format(**d))
    #
    # print("Time" * 5)
    # for d in info_time:
    #     print(ltx_ln.format(**d))


if __name__ == '__main__':
    main()
