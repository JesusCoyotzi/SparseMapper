import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import sys


def load_image(infilename):
    img = Image.open(infilename)
    img.load()
    data = np.asarray(img, dtype="float32")
    return data


def load_serialized(filename):
    data=np.zeros(1)
    with open(filename, 'r') as f:
        f.readline()
        size_str = f.readline()
        w = int(size_str.split()[1])
        h = int(size_str.split()[2])
        data=np.zeros([w,h])
        for line in f.readlines():
            splitted_line=line.split()
            idx=int(splitted_line[0])
            idy=int(splitted_line[1])
            map_val=int(splitted_line[2])
            if(map_val<0):
                map_val=125
            elif(map_val>0):
                map_val=255
            data[idx,idy]=map_val
    return data


def main(filename):
    map_img = load_serialized(filename)
    plt.imshow(map_img,cmap='gray')
    plt.show()
    return


if __name__ == '__main__':
    main(sys.argv[1])
