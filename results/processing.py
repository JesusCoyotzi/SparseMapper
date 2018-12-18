import sys
import pandas as pd
import os


def main():
    if len(sys.argv) < 2:
        print("Error usage:")
        print("processing.py folder_to_process method")
        return
    csv_path = os.path.abspath(sys.argv[1])

    files = [os.path.join(csv_path, f) for f in os.listdir(csv_path)]
    for file in files:
        df_metadata = pd.read_csv(file, skiprows=1, nrows=1, header=0)
        # if not (df_metadata.iloc[0]["method"] ==  mthd):
        #     continue
        simul_data = pd.read_csv(file, skiprows=4)
        a = simul_data["codebook"].unique()
        print(a)
        for i in a:
            cursor = simul_data.loc[simul_data["codebook"] == i]
            print(cursor.mean()["distorsion"])


if __name__ == '__main__':
    main()
