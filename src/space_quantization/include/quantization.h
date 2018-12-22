#include <stdio.h>
#include <unistd.h>
#include <cstdlib>
#include <cmath>
#include <cfloat>
#include <time.h>
#include <fstream>
#include <cuda.h>
#include <cuda_runtime.h>
#include <curand.h>
#include <curand_kernel.h>


//PCL structures have to much overhead and complexity,
//simply point structure defined to easy operation with cuda
typedef struct _point3 {
        //AoS
        float x;
        float y;
        float z;
} point3;

//operands
void printVec(float vec[], int n);
void printVec(int vec[], int n);
void printPoint3(point3 p);
void printPoint3Array(point3 *p, int n);
__device__ __host__ point3 addPoint3(point3 p1,point3 p2);
__device__ __host__ point3 mulPoint3(point3 p, float s);

//quantization

__device__ __host__ point3 addPoint3(point3 p1,point3 p2);


__global__ void setup_kernel(curandState *state);
__device__ point3 randomPoint3(curandState *randState);

__device__ __host__ float euclideanDistance(point3 p1, point3 p2);
__global__ void distanceKernel(point3 * points, point3 * centroids,
                               float * distances,
                               int k, int n );
__global__ void recalcCentroids(point3 *points,
                                point3 *centroids,
                                int * partition,
                                int k, int n);
__global__ void recalcCentroidsOuter(point3 * points,
                                     point3* centroids,
                                     int *histogram,
                                     int k, int n);
__global__ void recalcCentroidsOuter(point3 * points,
                                     point3* centroids,
                                     int *histogram,
                                     int k, int n,
                                     curandState *crs,
                                     point3 maxP,point3 minP);
__global__ void recalcCentroidsInner(point3 * points,
                                     point3* partialResult,
                                     int n);
__global__ void makePartition(int *partition,
                              float *distances, int *histogram,
                              int k, int n);
__global__ void prepareReduceArray(point3 *points,
                                   int *partition,
                                   point3 *reduceArrray,
                                   int centroid, int n);
bool kmeans(point3 *h_points, int *h_partition,
            point3* h_codebook, int *h_histogram,
            int iterations, int clusters, int nPoints);
bool kmeans(point3 *h_points, int *h_partition,
            point3* h_codebook, int *h_histogram,
            int iterations, int clusters, int nPoints,
            point3 maxP, point3 minP);
void printVec(int vec[], int n);
void randomInts(int arr[],int n);
point3 randomPoint3();
void printPoint3Array(point3 *p, int n);
void printPoint3(point3 p);
void serializeQuantization(point3* points, point3* codebook, int * partition, int n, int k, char *filename);
void initializeCodebook(point3 * points, point3 minPoint,
                        point3 maxPoint,int nPoints);
void initializeCodebook(point3 * codebook, point3* points,
                        int pointSize, int nClusters);
//Kmeans ++
void kppInitCPU(point3 *points,  point3 *codebook,
                int clusters, int nPoints);
//cosas lbgosas
bool LBGCPU(point3 *points,  point3 *codebook,
            int *histogram, int *partition,
            int iterations, int clusters, int nPoints);
void perturbate(point3 *codes,point3 *prev,int n);
void getPartition(point3 *codebook, point3 *points,
                  int* partition, int* histogram,
                  int nPoints, int clusters);
void recalcCentroids(point3 *codebook, point3 *points,
                     int* partition, int* histogram,
                     int nPoints, int clusters);
void copyCodebook(point3 * prev, point3 * cdbk, int nClusters);
point3 getCentroid(point3 * points, int n);
