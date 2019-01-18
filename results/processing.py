import sys
import pandas as pd
import os

#import seaborn as sns
import matplotlib.pyplot as plt

from collections import defaultdict
#from sets import Set


def getFilesInTree(pth):
    csvs = []
    for dirname, subdirs, files in os.walk(pth):
        print("In directory: %s" % dirname)
        # print("Subdirs")
        # for dir in subdirs:
        #     print("\t%s " % dir)

        for fil in files:
            filePath = os.path.join(os.path.abspath(dirname), fil)
            csvs.append(filePath)
    return csvs


def readHeaders(csv_files):
    cloud_dict = defaultdict(list)

    for csv_file in csv_files:
        df_metadata = pd.read_csv(csv_file, skiprows=1, nrows=1, header=0)
        cloud_file = os.path.basename(df_metadata["filename"])
        cloud_dict[cloud_file].append(csv_file)

    return cloud_dict

def reduceFrame(valid_data,n_points,meth):
    cltrs = valid_data["requested"].unique()
    local_data = []
    for i in cltrs:
        cursor = valid_data.loc[valid_data["clusters"] == i]
        avg_dist = cursor.mean()["distorsion"] / n_points
        avg_time = cursor.mean()["time"]
        avg_stddev = cursor.mean()["stddev"]

        tmp_dic = {"clusters": i,
                   "time": avg_time,
                   "distorsion": avg_dist,
                   "stddev": avg_stddev,
                   "method": meth,
                   "points": n_points}
        local_data.append(tmp_dic)
    # print(local_frame)
    local_frame = pd.DataFrame(local_data)
    return local_frame


def makeSummary(csv_files):
    while not csv_files:
        f = csv_file.pop()
        df_metadata = pd.read_csv(f, skiprows=1, nrows=1, header=0)
        cloud_file = os.path.basename(df_metadata["filename"])
        for csv in csv_files:
            df_metadata = pd.read_csv(csv, skiprows=1, nrows=1, header=0)
            if cloud_file == os.path.basename(df_metadata["filename"]):
                pass
        return


def getFileName(csv_file):
    print(csv_file)
    df_metadata = pd.read_csv(csv_file, skiprows=1, nrows=1, header=0)
    cloud_file = os.path.basename(df_metadata["filename"][0])
    return os.path.splitext(cloud_file)[0]


def processFiles(csv_file):
    df_metadata = pd.read_csv(csv_file, skiprows=1, nrows=1, header=0)

    # if not (df_metadata.iloc[0]["method"] ==  mthd):
    #     continue
    meth = df_metadata["method"][0]
    simul_data = pd.read_csv(csv_file, skiprows=4)
    #print(csv_file)
    valid_data = simul_data.loc[simul_data["clusters"]
                                == simul_data["requested"]]
    n_points = df_metadata["points"][0]

    # local_frame = reduceFrame(valid_data,n_points,meth)
    cltrs = valid_data["requested"].unique()
    local_data = []
    for i in cltrs:
        cursor = valid_data.loc[valid_data["clusters"] == i]
        avg_dist = cursor.mean()["distorsion"] / n_points
        avg_time = cursor.mean()["time"]
        avg_stddev = cursor.mean()["stddev"]

        tmp_dic = {"clusters": i,
                   "time": avg_time,
                   "distorsion": avg_dist,
                   "stddev": avg_stddev,
                   "method": meth,
                   "points": n_points}
        local_data.append(tmp_dic)
    # print(local_frame)
    local_frame = pd.DataFrame(local_data)
    return local_frame


def makeSummaryFigure(df, output_name):
    plt.style.use("seaborn-whitegrid")
    methods = ("kmeansCPU", "inner", "LBG", "kpp")
    f_time, ax_time = plt.subplots()
    f_dist, ax_dist = plt.subplots()
    f_dev, ax_dev = plt.subplots()

    ax_time.set_title("Average Time")
    ax_time.set_xlabel("Number of clusters")
    ax_time.set_ylabel("Time [s]")

    ax_dist.set_title("Mean Squared Distorsion")
    ax_dist.set_xlabel("Number of clusters")
    ax_dist.set_ylabel("Meand Distorsion [m]")

    ax_dev.set_title("Average standard deviation")
    ax_dev.set_xlabel("Number of clusters")
    ax_dev.set_ylabel("Standard deviation [clusters]")

    for met in methods:
        df_mth = df.loc[df["method"] == met]
        time_series = df_mth["time"]
        distorsion_series = df_mth["distorsion"]
        stddev_series = df_mth["stddev"]
        cluster_series = df_mth["clusters"]

        ax_time.plot(cluster_series, time_series, label=met)
        ax_dist.plot(cluster_series, distorsion_series, label=met)
        ax_dev.plot(cluster_series, stddev_series, label=met)

    ax_time.legend(loc='best')
    ax_dist.legend(loc='best')
    ax_dev.legend(loc='best')
    f_time.savefig(output_name+"-time.png",dpi=300)
    f_dist.savefig(output_name+"-dist.png",dpi=300)
    f_dev.savefig(output_name+"-dev.png",dpi=300)
# plt.show()


def processFolder(folder):
    files = [os.path.join(folder, f) for f in os.listdir(folder)]
    #print(files)
    cols = ["clusters", "time", "distorsion", "stddev", "method","points"]
    df_summary = pd.DataFrame(columns=cols)
    pcd_file = getFileName(files[0])
    for fil in files:
        if pcd_file == getFileName(fil):
            summary = processFiles(fil)
            df_summary = df_summary.append(summary, ignore_index=True)
    print(df_summary)
    #output_name = os.path.join(csv_path, pcd_file + ".csv")
    # df_summary.to_csv(os.path.join(output_folder, pcd_file + ".csv"))
    # makeSummaryFigure(df_summary,output_name)
    return df_summary, pcd_file


def saveCsvAndFig(df_summary, output_folder, pcd_file):
    csv_name = os.path.join(output_folder, pcd_file + ".csv")
    image_name = os.path.join(output_folder, pcd_file)
    df_summary.to_csv(csv_name)
    makeSummaryFigure(df_summary, image_name)


def main():
    if len(sys.argv) < 3:
        print("Error usage:")
        print("processing.py folder_to_process output")
        return
    csv_folder = os.path.abspath(sys.argv[1])
    output_folder = os.path.abspath(sys.argv[2])
    pcd_folders = [os.path.join(csv_folder, dI) for dI in os.listdir(
        csv_folder) if os.path.isdir(os.path.join(csv_folder, dI))]
    #print(pcd_folders)
    for pcd_folder in pcd_folders:
        df_sum, pcd_name = processFolder(pcd_folder)
        saveCsvAndFig(df_sum, output_folder, pcd_name)

    # processFolder(csv_path)


if __name__ == '__main__':
    main()
