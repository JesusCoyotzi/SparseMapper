#include <stdio.h>
#include <unistd.h>
#include <cstdlib>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cmath>
#include <cfloat>
#include <time.h>
#include <fstream>


//PCL structures have to much overhead and complexity,
//simply point structure defined to easy operation with cuda
typedef struct _point3 {
        //AoS
        float x;
        float y;
        float z;
} point3;


// typedef struct _points3
// {
//         //SoA
//         float x[];
//         float y[];
//         float z[];
// } points3;

__device__ float euclideanDistance(point3 p1, point3 p2);
__global__ void distanceKernel(point3 * points, point3 * centroids,
                               float * distances,
                               int k, int n );
__global__ void recalcCentroids(point3 *points,
                                point3 *centroids,
                                int * partition,
                                int k, int n);
__global__ void recalcCentroidsOuter(point3 * points,
                                     point3* partialResult,
                                     point3* centroids,
                                     int *partition,
                                     int *partitionOut,
                                     int k, int n);
__global__ void recalcCentroidsInner(point3 * points,
                                     point3* sumOut,
                                     point3* centroids,
                                     int *partition,
                                     int *partitionOut,
                                     int k, int n);
__global__ void makePartition(int *partition,
                              float *distances,
                              int k, int n);
void kmeans(point3 *h_points, int *h_partition,
            point3* h_codebook,
            int iterations, int clusters, int nPoints);
void printVec(int vec[], int n);
void randomInts(int arr[],int n);
void kmeans(point3 *h_points, point3 *h_out, point3* h_codebook,
            int iterations, int clusters, int nPoints);
point3 randomPoint3();
void printPoint3Array(point3 *p, int n);
void printPoint3(point3 p);
void serializeQuantization(point3* points, point3* codebook, int * partition, int n, int k, char *filename);
void initializeCodebook(point3 * points, point3 minPoint,
                       point3 maxPoint,int nPoints);
