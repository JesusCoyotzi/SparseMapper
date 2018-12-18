import sys
import pandas as pd
import os


def main():
    if len(sys.argv) < 2:
        print("Error usage:")
        print("processing.py file_to_process")
        return
    csv_path = os.path.abspath(sys.argv[1])
    df_metadata = pd.read_csv(csv_path, skiprows=1, nrows=1, header=0)
    # if not (df_metadata.iloc[0]["method"] ==  mthd):
    #     continue
    cols = ["time","distorsion"]
    df_avg = pd.DataFrame(columns=cols)
    df_avg.rename_axis("clusters",inplace=True)

    simul_data = pd.read_csv(csv_path, skiprows=4)
    codebooks = simul_data["clusters"].unique()
    for code in codebooks:
        cursor = simul_data.loc[simul_data["clusters"] == code]
        cadena = "With {} clusters \n\t distorsion:{} in {} secs"
        avg_dist = cursor.mean()["distorsion"]
        avg_time = cursor.mean()["time"]
        cadena=cadena.format(code,avg_dist , avg_time)
        print(cadena)
        tmp_dict = {"time":avg_time, "distorsion":avg_dist}
        df_avg.loc[code] = tmp_dict

    print(df_avg.head())
    base_name = os.path.splitext(csv_path)[0]
    out_name = base_name+"avg.csv"
    df_avg.to_csv(out_name)
        # print("With {} clusters ")
        # print(cursor.mean()["distorsion"])
        # print(cursor.mean()["time"])


if __name__ == '__main__':
    main()
