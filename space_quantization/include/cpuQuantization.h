#include <stdio.h>
#include <unistd.h>
#include <cstdlib>
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

//operands
void printVec(float vec[], int n);
void printVec(int vec[], int n);
void printPoint3(point3 p);
void printPoint3Array(point3 *p, int n);
point3 addPoint3(point3 p1,point3 p2);
point3 mulPoint3(point3 p, float s);

//quantization
point3 addPoint3(point3 p1,point3 p2);

float euclideanDistance(point3 p1, point3 p2);

bool kmeansCPU(point3 *points, int *partition,
               point3 *codebook, int *histogram,
               int iterations, int clusters, int nPoints);


void serializeQuantization(point3* points, point3* codebook, int * partition, int n, int k, char *filename);
void initializeCodebook(point3 * points, point3 minPoint,
                        point3 maxPoint,int nPoints);
void initializeCodebook(point3 * codebook, point3* points,
                        int pointSize, int nClusters);
//Kmeans ++
void kppInitCPU(point3 *points,  point3 *codebook,
                int clusters, int nPoints);

// LBG related functions 
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
