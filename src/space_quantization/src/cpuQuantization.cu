#include "quantization.h"

void kppInitCPU(point3 *points,  point3 *codebook,
                int clusters, int nPoints)
{
        //kemeans ++ initialization
        printf("Generating %d clusters for %d points\n", clusters, nPoints );
        srand(time(NULL));
        int rndIdx = rand() % nPoints;
        codebook[0]= points[rndIdx];
        printf("Cluster  0 is point[%d] ",rndIdx );
        printPoint3(codebook[0]);


        float * dist   =  (float *) malloc(nPoints*sizeof(float));

        //Compute dist to all points


        for (size_t j = 1; j < clusters; j++)
        {
                float acc = 0.0;
                // printf("Prev Codebook" );
                // printPoint3(codebook[j-1]);
                for (size_t i = 0; i < nPoints; i++)
                {
                        //printf("[%d]\n",i);
                        dist[i]=euclideanDistance(codebook[j-1],points[i]);
                        acc+=dist[i];

                }
                //Sampling wheel
                float r = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/acc));
                int k = 0;
                while (r>0.0)
                {
                        r = r - dist[k];
                        k++;
                }
                codebook[j]=points[k-1];
                // printf("Cluster %d is point[%d] ",j,k );
                // printPoint3(codebook[j]);

        }

        free(dist);
        return;
}

void LBGCPU(point3 *points,  point3 *codebook,
            int *histogram, int *partition,
            int iterations, int clusters, int nPoints)
{
        //CPu code to run LBG
        bool isPowerOfTwo = (clusters & (clusters - 1)) == 0;
        if (!isPowerOfTwo) {
                printf("Error clusters must be power of 2 2,4,8,16..\n" );
                return;
        }
        point3 * prevCodebook;
        prevCodebook = (point3 *) malloc(sizeof(point3)*clusters);
        prevCodebook[0]=getCentroid(points,nPoints);
        for (int i = 1; i < clusters; i=i<<1)
        {
                printf("Working with %d clusters\n",i);
                perturbate(codebook,prevCodebook,i);
                printf("Codebook\n");
                printPoint3Array(prevCodebook,i);
                for (int j = 0; j < iterations; j++)
                {
                        //  printf("\tIteration [%d]\n",j );
                        getPartition(codebook,points, partition, histogram, nPoints,i*2);
                        recalcCentroids(codebook, points, partition, histogram,nPoints,i*2);
                }
                //copy centroids to prevCodebook
                copyCodebook(prevCodebook,codebook,clusters);
        }
        free(prevCodebook);

        return;

}

void copyCodebook(point3 * prev, point3 * cdbk, int nClusters)
{
        //Copy codebook prev into cdbk
        for (size_t i = 0; i < nClusters; i++) {
                prev[i]=cdbk[i];
        }
        return;
}


void perturbate(point3 *codes,point3 *prev,int n)
{
        const float e = 0.01;
        point3 epsilon_plus; epsilon_plus.x=e; epsilon_plus.y=e; epsilon_plus.z=e;
        point3 epsilon_min; epsilon_min.x=-e; epsilon_min.y=-e; epsilon_min.z=-e;
        //int nextN = n<<1;
        int j=0;
        for (int i = 0; i < n; i++) {
                codes[j] = addPoint3(prev[i],epsilon_plus);
                codes[j+1] = addPoint3(prev[i],epsilon_min);
                j+=2;
        }
        return;
}

void getPartition(point3 *codebook, point3 *points,
                  int* partition, int * histogram,
                  int nPoints, int clusters)
{
        //Get the points partition
        for (size_t i = 0; i < clusters; i++) {
                histogram[i]=0;
        }
        for (int j = 0; j < nPoints; j++)
        {
                partition[j] = 0;
                float minDist = euclideanDistance(codebook[0],points[j]);
                for (int k = 1; k < clusters; k++)
                {
                        float dist = euclideanDistance(codebook[k],points[j]);
                        if (minDist>dist)
                        {
                                minDist=dist;
                                partition[j]=k;
                        }
                }
                histogram[partition[j]]++;
        }
        // for (int i = 0; i < clusters; i++) {
        //         printf("Cluster[%d]: %d\n",i,histogram[i] );
        // }
        return;
}

point3 getCentroid(point3 * points, int n)
{
        point3 acc;
        acc.x=0.0; acc.y=0.0; acc.z=0.0;
        for (int i = 0; i < n; i++) {
                acc.x+=points[i].x;
                acc.y+=points[i].y;
                acc.z+=points[i].z;
        }
        point3 centroid = mulPoint3(acc,1.0/n);
        printf("The centroid is [%f,%f,%f] \n",centroid.x,centroid.y,centroid.z);
        return centroid;
}