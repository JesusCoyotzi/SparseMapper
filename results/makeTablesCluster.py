import pandas as pd
import sys
import os

def main():
    if len(sys.argv) < 2:
        print ("usage: program file")
        return
    cluster_df = pd.read_csv(sys.argv[1], index_col=0)
    cluster_tupl = (16,32,64,128,256,512)
    mtd_tpl = ("kmeansCPU","inner","kpp","LBG")
    ltx_ln =  " & {clusters} & {kmeansCPU:.4f}  & {inner:.4f}  & {LBG:.4f}  & {kpp:.4f} \\\\"
    info_dist =[]
    info_time =[]
    for clt  in cluster_tupl:
        clt_msk = cluster_df["clusters"] == clt
        nodes_df= cluster_df[clt_msk]
        dic_dist = {"clusters":clt}
        dic_time = {"clusters":clt}
        for method in mtd_tpl:
            method_msk = nodes_df["method"] == method
            mtd_df = nodes_df[method_msk]
            dic_dist[method] = mtd_df["distorsion"].iloc[0]
            dic_time[method] = mtd_df["time"].iloc[0]
        info_dist.append(dic_dist)
        info_time.append(dic_time)


    print("Distorsion"*5)
    print(" clusters \t kmeansCPU \t inner \t LBG \t kpp")
    for d in info_dist:
        print(ltx_ln.format(**d))

    print("Time"*5)
    print(" clusters \t kmeansCPU \t inner \t LBG \t kpp")
    for d in info_time:
        print(ltx_ln.format(**d))



if __name__ == '__main__':
    main()
