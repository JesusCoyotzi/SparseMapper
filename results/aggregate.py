import pandas as pd
import sys
import os

# find `pwd`/rgbd/ `pwd`/nyu/ `pwd`/heavyClouds/ -maxdepth 1 -name "*.csv" > agg.txt


def readFilesToAgg(path_to_files):
    with open(path_to_files) as f:
        files = f.read().splitlines()
        return files


def main():
    if len(sys.argv) < 3:
        print("Error usage:")
        print("aggregate.py files_file output_file")
        return


    csv_file_list = os.path.abspath(sys.argv[1])
    aggregated_file = os.path.abspath(sys.argv[2])
    files_to_agg = readFilesToAgg(csv_file_list)
    print (files_to_agg)
    df_agg = pd.DataFrame()
    for f in files_to_agg:
        f_name = os.path.basename(f)
        df_tmp = pd.read_csv(f,index_col=0)
        df_tmp = df_tmp.assign(filename=f_name)
        df_agg=df_agg.append(df_tmp,ignore_index=True)
        #print(df_tmp.head())

    print(df_agg["filename"].unique())
    df_agg.to_csv(aggregated_file)


if __name__ == '__main__':
    main()
