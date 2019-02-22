# To make figures of kmneas and LBG

import numpy as np
#import open3d
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


def is_power2(num):

	'states if a number is a power of two'

	return num != 0 and ((num & (num - 1)) == 0)


def lloyd(space, centroids,  iterations=7):
    #    centroids = np.random.choice(space)
    # gets len(centroids) clusters
    for i in range(iterations):
        dist = np.linalg.norm(space - centroids[:, np.newaxis], axis=2)
        partition = np.argmin(dist, axis=0)
        centroids = np.array([space[partition == k].mean(axis=0)
                              for k in range(centroids.shape[0])])
    return partition, centroids


def LBG(space, clusters, iterations=10):
    if not is_power2(clusters):
        print("Error non power of two")
        return
    ini_centroid = space.mean(axis=0)
    epsilon = np.array([[0.01, 0.01], [-0.01, -0.01]])
    centroids = ini_centroid + epsilon
    while len(centroids) < clusters:
        _, centroids = lloyd(space, centroids, iterations)
        #print(centroids)
        centroids = np.vstack((centroids + epsilon[0], centroids + epsilon[1]))

    partition, centroids = lloyd(space, centroids, iterations)
    return partition, centroids


def plotPartition(space, centroids, partition):
    X = space[:, 0]
    Y = space[:, 1]
    c_x = centroids[:, 0]
    c_y = centroids[:, 1]

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.axis("equal")

    ax.scatter(X, Y, c=partition, s=1, cmap="jet")
    ax.scatter(c_x, c_y, c="r", s=10, marker="s", edgecolors="blue")
    ax.set_title("Cuantización Vectorial")
    return


def main2d():
    sz = 10000
    i = 10
    k = 64

    mu = [0, 0]
    cov = [[4, 0], [0, 4]]
    #space = np.random.multivariate_normal(mu, cov, sz)
    space = np.random.uniform(-3, 5, size=(sz, 2))
    #print(space[0:4])
    idx = np.random.choice(len(space), k, replace="false")
    initial_centroids = space[idx]
    #partition, centroids = lloyd(space, initial_centroids, i)
    partition, centroids = LBG(space, k, i)
    plotPartition(space, centroids, partition)
    # partition,centroids = lloyd(space,centroids,k,i)
    # plotPartition(space,centroids,partition)

    # Graphing
    plt.show()


def main3d():
    sz = 1000
    i = 5
    k = 5

    mu = [0, 0, 0]
    cov = [[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]]
    space = np.random.multivariate_normal(mu, cov, sz)
    # print(space[0])
    idx = np.random.choice(len(space), k)
    initial_centroids = space[idx]
    partition, centroids = lloyd(space, initial_centroids, k, i)
    # print(partition[0:5])
    # Graphing
    X = space[:, 0]
    Y = space[:, 1]
    Z = space[:, 2]
    c_x = centroids[:, 0]
    c_y = centroids[:, 1]
    c_z = centroids[:, 2]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    max_range = np.array([X.max() - X.min(), Y.max() -
                          Y.min(), Z.max() - Z.min()]).max() / 2.0
    mid_x = (X.max() + X.min()) * 0.5
    mid_y = (Y.max() + Y.min()) * 0.5
    mid_z = (Z.max() + Z.min()) * 0.5

    ax.scatter(X, Y, Z, c=partition, s=1, cmap="jet")
    ax.scatter(c_x, c_y, c_z, c="black", s=10, marker="s")
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    plt.show()


if __name__ == '__main__':
    plt.style.use("seaborn-whitegrid")

    main2d()


# x = np.random.normal(0, 1, sz)
# y = np.random.normal(0, 2, sz)
# z = np.random.normal(0, 1.5, sz)
# space = np.column_stack((x, y, z))
