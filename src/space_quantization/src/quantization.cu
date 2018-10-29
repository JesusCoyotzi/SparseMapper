#include "quantization.h"
#define THREADS 1024
#define MAX_CENTROIDS 1024

void printVec(float vec[], int n)
{
        printf("[");
        printf("%f",vec[0]);
        for(int i= 1; i<n; i++)
        {
                printf(",%f",vec[i]);
        }
        printf("]\n");
        return;
}


void printVec(int vec[], int n)
{
        printf("[");
        printf("%d",vec[0]);
        for(int i= 1; i<n; i++)
        {
                printf(",%d",vec[i]);
        }
        printf("]\n");
        return;
}

void printPoint3(point3 p)
{
        printf("[%f,%f,%f]\n",p.x,p.y,p.z);
        return;
}

void printPoint3Array(point3 *p, int n)
{
        for(int i=0; i<n; i++)
        {
                printPoint3(p[i]);
        }
}

__device__ point3 addPoint3(point3 p1,point3 p2)
{
        point3 sum;
        sum.x=p1.x+p2.x;
        sum.y=p1.y+p2.y;
        sum.z=p1.z+p2.z;

        return sum;
}

__device__ point3 mulPoint3(point3 p, float s)
{
        point3 mul;
        mul.x=s*p.x;
        mul.y=s*p.y;
        mul.z=s*p.z;

        return mul;
}

point3 randomPoint3()
{
        float x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

        point3 randomPoint;
        const float min_z =0.8;
        const float max_z =3;
        const float min_x = -1.5, max_x =1.5;
        const float min_y = -1.5, max_y =1.5;
        randomPoint.x = (max_x-min_x)*x+min_x;
        randomPoint.y = (max_y-min_y)*y+min_y;
        randomPoint.z = (max_z-min_z)*z+min_z;
        return randomPoint;

}

point3 randomPoint3(point3 min, point3 max)
{
        //generates pseudorandom number between 0 and 1
        float x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

        // shift and center around the cloud
        point3 randomPoint;
        randomPoint.x = (max.x-min.x)*x+min.x;
        randomPoint.y = (max.y-min.y)*y+min.y;
        randomPoint.z = (max.z-min.z)*z+min.z;
        return randomPoint;

}

void initializeCodebook(point3 * codebook, point3 minPoint,
                        point3 maxPoint,int nClusters)
{
        //loop over all clusters
        printf("Initializing centroids\n");
        printf("Max point:\t"); printPoint3(maxPoint);
        printf("\nMin point:\t"); printPoint3(minPoint);
        printf("\n");
        srand(time(NULL));

        for (unsigned int i = 0; i < nClusters; i++)
        {
                codebook[i] = randomPoint3(minPoint,maxPoint);
                //printPoint3(codebook[i]);
        }
        return;
}

__device__ float euclideanDistance(point3 p1, point3 p2)
{
        return ( (p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z));
}

__device__ void setZerosDevice(point3 *p,int n)
{
        //sets an array of points with only 0
        point3 zero;
        zero.x=0; zero.y=0; zero.z=0;
        for(int i = 0; i < n; i++)
        {
                p[i]=zero;
        }

        return;
}

__device__ void setZerosDevice(int *p,int n)
{
        //sets an array of points with only 0
        int zero = 0;
        for(int i = 0; i < n; i++)
        {
                p[i]=zero;
        }

        return;
}

__global__ void distanceKernel(point3 * points, point3 * centroids,float * distances, int k, int n )
{
        //distances[k*n]
        /*The distance array is a 2d array but i flattened it into a one dimensional array cause i am
           to lazy to create a 2d array on cuda. */
        int idx = threadIdx.x + blockIdx.x * blockDim.x;
        if (idx < n) {

                for(int i= 0; i< k; i++)
                {
                        //distances[point,centroid]
                        distances[n*i+idx]=euclideanDistance(points[idx],centroids[i]);
                        //distance[centroid, point]
                        //distances[i+k*idx]=euclideanDistance(points[idx],centroids[i]);
                }
        }
        return;
}

__global__ void makePartition(int *partition,
                              float *distances, int *histogram,
                              int k, int n)
{
        int idx  = threadIdx.x + blockIdx.x * blockDim.x;
        //__shared__ int sharedPartition[THREADS];
        //setZerosDevice(histogram,k);
        //__syncthreads();
        if (idx < n) {
                //int tid = threadIdx.x;
                int minixd=0;
                float minDist=distances[idx];
                for(int i=1; i<k; i++)
                {
                        if(minDist>distances[idx+n*i])
                        {
                                minixd=i;
                                minDist=distances[idx+n*i];
                        }
                }
                partition[idx]=minixd;
                atomicAdd(&histogram[minixd],1);
        }
        // __syncthreads();
        // if (idx==0) {
        //   printf("-----\n" );
        //         for (int i = 0; i < k; i++) {
        //                 printf("Histogram[%d]=%d\n",i,histogram[i] );
        //         }
        // }
        return;
}

__global__ void prepareReduceArray(point3 *points, int *partition, point3 *reduceArrray, int centroid, int n)
{
        //takes the whole partition array and outputs a single array that only has points belonging to a
        //particular centroid
        int idx = blockIdx.x*blockDim.x+threadIdx.x;
        if(idx < n)
        {
                //set all to 0;
                //copy to shared if assigned to jth centroid
                if(partition[idx]==centroid)
                {
                        //only copy from partition j
                        reduceArrray[idx]=points[idx];
                }
                else
                {
                        point3 zero;
                        zero.x=0; zero.y=0; zero.z=0;
                        reduceArrray[idx]=zero;
                }
        }
        return;
}

__global__ void recalcCentroidsInner(point3 * points,
                                     point3* partialResult,
                                     int *partition,
                                     int n)
{
        //n = histogram[n]
        __shared__ point3 pointsShared[THREADS];
        //__shared__ int partitionShared[THREADS];
        int idx = blockIdx.x*blockDim.x+threadIdx.x;
        if(idx < n)
        {
                int tid = threadIdx.x;
                //set all to 0;
                setZerosDevice(pointsShared,THREADS); //Size of shared memory
                //setZerosDevice(partitionShared,THREADS); //Size of shared memory

                __syncthreads();
                //only copy from partition j
                pointsShared[tid]=points[idx];
                //makes two arrays [k k k 0 0 0 0 k k k]
                //[1 1 1 0 0 0 0 1 1 1]
                /*Array of valid points and zeros and array of ones*/
                __syncthreads();
                for (int s = blockDim.x/2; s>0; s>>=1)
                {
                        if(tid<s)
                        {
                                {
                                        pointsShared[tid]=addPoint3(
                                                pointsShared[tid],
                                                pointsShared[tid+s]);

                                }
                        }
                        __syncthreads();
                }
                if(tid==0) {
                        //printf("Block: %d\n",blockIdx.x);
                        //printf("Number of elements in  %d is %d\n",k, partitionShared[0]);
                        //partitionOut[blockIdx.x] = partitionShared[0];
                        partialResult[blockIdx.x] = pointsShared[0];
                        //centroids[k] = mulPoint3(pointsShared[0],1.0/partitionShared[0]);
                }
        }
        __syncthreads();
}

__global__ void recalcCentroidsOuter(point3 * points,
                                     point3* centroids,
                                     int *partition,
                                     int *histogram,
                                     int k, int n)
{
        //histogram is a count of how many elements belong to each centroid
        __shared__ point3 pointsShared[THREADS];
        int idx = blockIdx.x*blockDim.x+threadIdx.x;
        if(idx < n)
        {
                int tid = threadIdx.x;

                //set all to 0; //El error estaba en alocar n cachos
                setZerosDevice(pointsShared,THREADS); //Size of shared memory
                __syncthreads();

                pointsShared[tid]=points[idx];
                __syncthreads();
                for (int s = blockDim.x/2; s>0; s>>=1)
                {
                        if(tid<s)
                        {
                                {
                                        pointsShared[tid]=addPoint3(
                                                pointsShared[tid],
                                                pointsShared[tid+s]);

                                }
                        }
                        __syncthreads();
                }
                if(tid==0)
                {
                        //printf("Thread: %d\t",idx);
                        //printf("[Inner] Number of elements in  %d is %d\n",k, partitionShared[0]);
                        // printf("Centroid[%d] prior:%f,%f,%f\n",
                        //        k,centroids[k].x,centroids[k].y,centroids[k].z);
                        if(histogram[k]>0) {
                                centroids[k] = mulPoint3(pointsShared[0],1.0/histogram[k]);
                        }
                }
        }
        __syncthreads();
}

__global__ void partitionToLocal(point3 *points,
                                 point3 *partitionPoints,
                                 //point3* centroids,
                                 int *partition,
                                 int *localPartition,
                                 int k, int n)
{
        __shared__ point3 pointsShared[THREADS];
        __shared__ int partitionShared[THREADS];
        int idx = blockIdx.x*blockDim.x+threadIdx.x;
        if(idx < n)
        {
                int tid = threadIdx.x;

                //set all to 0;
                setZerosDevice(pointsShared,THREADS); //Size of shared memory
                setZerosDevice(partitionShared,THREADS); //Size of shared memory
                //copy to shared if assigned to jth centroid
                __syncthreads();
                if(partition[idx]==k)
                {
                        //only copy from partition j
                        pointsShared[tid]=points[idx];
                        partitionShared[tid]=1;
                } //makes two arrays [j j j 0 0 0 0 j j j]
                __syncthreads();

                partitionPoints[idx]=pointsShared[tid];
                localPartition[idx]=partitionShared[tid];
        }
}

void initializeCentroids(point3 *points, point3 aleatorios)
{
        return;
}
//points todos los puntos
//aleatorios vectores aleatorios para perturbar el centroide

void kmeans(point3 *h_points, int *h_partition,
            point3* h_codebook,
            int iterations, int clusters, int nPoints)
{

        printf("Received: %d\n",nPoints);

        //Pointers
        point3 *d_points, *d_codebook, *d_partialSum;
        float *d_distances,*h_distances;
        int *d_partition, *d_partialPart, *d_histogram;
        point3 *d_reduceArray;
        //sizes
        int nPointsSize   = nPoints*sizeof(point3);
        int clustersSize  = clusters*sizeof(point3);
        int distanceSize  = nPoints*clusters*sizeof(float);
        int partitionSize = nPoints*sizeof(int);
        int histogramSize = clusters*sizeof(int);

        h_distances = (float *) malloc(distanceSize);
        //h_partition = (int *) malloc(partitionSize);

        cudaMalloc((void**)&d_points,nPointsSize);
        cudaMalloc((void**)&d_codebook,clustersSize);
        cudaMalloc((void**)&d_distances,distanceSize);
        cudaMalloc((void**)&d_partition,partitionSize);
        cudaMalloc((void**)&d_partialSum,THREADS*sizeof(point3));
        cudaMalloc((void**)&d_partialPart,THREADS*sizeof(int));

        cudaMalloc((void**)&d_reduceArray,nPointsSize);
        cudaMalloc((void**)&d_histogram,histogramSize);

        cudaMemcpy(d_points,h_points,nPointsSize,cudaMemcpyHostToDevice);
        cudaMemcpy(d_codebook,h_codebook,clustersSize,cudaMemcpyHostToDevice);

        int blks = (nPoints + THREADS - 1) / THREADS;
        ///if blks > THREADS return error not enough kenerls
        //int blks = nPoints/ THREADS;
        printf("Issuing %d blocks with %d threads\n",blks, THREADS);
        //cudaDeviceSynchronize();
        // recalcCentroids<<<(nPoints + THREADS - 1) / THREADS,THREADS>>>
        // (d_points,d_codebook,d_partition,clusters,nPoints);
        for (int m = 0; m <iterations; m++) {
                distanceKernel<<<blks,THREADS>>>
                (d_points,d_codebook,d_distances,clusters,nPoints);
                //cudaDeviceSynchronize();
                cudaMemset(d_histogram, 0, histogramSize);
                makePartition<<<blks,THREADS>>>
                (d_partition,d_distances,d_histogram,clusters,nPoints);
                for(int i= 0; i<clusters; i++)
                {
                        prepareReduceArray<<<blks,THREADS>>>
                        (d_points,d_partition,d_reduceArray,i,nPoints);
                        //aqui iria un while
                        recalcCentroidsInner<<<blks,THREADS>>>
                        (d_reduceArray,d_reduceArray,d_partition,nPoints);
                        //Aqui acabaria el while
                        recalcCentroidsOuter<<<1,THREADS>>> //accccesing ilegal meory ?
                        (d_reduceArray,d_codebook,d_partition,d_histogram,i,blks);
                        if (cudaPeekAtLastError() != cudaSuccess) {
                                printf("kernel launch error: %s\n", cudaGetErrorString(cudaGetLastError()));
                        }

                }
        }
        cudaDeviceSynchronize();
        cudaMemcpy(h_distances,d_distances,
                   distanceSize,cudaMemcpyDeviceToHost);
        cudaMemcpy(h_partition,d_partition,
                   partitionSize,cudaMemcpyDeviceToHost);
        cudaMemcpy(h_codebook,d_codebook,
                   clustersSize,cudaMemcpyDeviceToHost);
        cudaMemcpy(h_partition,d_partition,
                   partitionSize,cudaMemcpyDeviceToHost);
        //printVec(h_partition,nPoints);
        //printVec(h_distances,nPoints*clusters);
        //printVec(h_partition,nPoints);
        printf("---Optimized centroids---\n");
        printPoint3Array(h_codebook, clusters);
        cudaFree(d_points); cudaFree(d_distances); cudaFree(d_codebook);
        cudaFree(d_partition); cudaFree(d_partialSum); cudaFree(d_partialPart);

        free(h_distances);
        return;
}

void serializeQuantization(point3* points, point3* codebook,
                           int * partition, int n, int k, char *filename)
{
        //points    Elements to be serialized
        //codebook  Final vectors
        //partition Correspondence
        //element points[i] belongs to codebook[partition[i]]
        //n number of points, k number of clusters
        printf("Writing to %s",filename);
        std::ofstream vqOut;
        vqOut.open(filename);
        vqOut<<"Points: " <<n << "\n";
        vqOut<<"Clusters: " <<k << "\n";
        vqOut<<"Codebook:\n";
        for(int i=0; i<k; i++)
        {
                vqOut<<codebook[i].x<<",";
                vqOut<<codebook[i].y<<",";
                vqOut<<codebook[i].z<<"\n";
        }
        vqOut<<"Points:\n";
        for(int i=0; i<n; i++)
        {
                vqOut<<points[i].x<<",";
                vqOut<<points[i].y<<",";
                vqOut<<points[i].z<<",";
                vqOut<<partition[i]<<"\n";
        }
        vqOut.close();
        return;

}


__global__ void reducePoint3(point3 *cloud_in, point3 *cloud_out)
{
        __shared__ point3 pointsShared[THREADS];
        int idx = blockIdx.x*blockDim.x+threadIdx.x;
        int tid = threadIdx.x;
        pointsShared[tid]=cloud_in[idx];
        __syncthreads();

        for (int s = blockDim.x/2; s>0; s>>=1)
        {
                if(tid<s)
                {
                        pointsShared[tid]=addPoint3(
                                pointsShared[tid],
                                pointsShared[tid+s]);

                }
                __syncthreads();
        }


        if(tid==0) cloud_out[blockIdx.x]=pointsShared[0];
}

int getFreeMem()
{
        size_t *libre=NULL, *ocupada=NULL; //sí se me acabo el ingles
        cudaMemGetInfo(libre,ocupada);
        return *libre;
}
