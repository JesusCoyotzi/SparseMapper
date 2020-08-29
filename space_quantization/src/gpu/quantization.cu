#include "quantization.h"
#define THREADS 1024
#define MAX_CENTROIDS 1024


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

__global__ void setup_kernel(curandState *state)
{
        int idx = threadIdx.x+blockDim.x*blockIdx.x;
        curand_init(1234, idx, 0, &state[idx]);
}

__device__ point3 randomPoint3(curandState *randState,point3 maxP,point3 minP)
{
        //REturn a random number from 0-1 meters in every direction
        point3 ranP3;
        ranP3.x=(maxP.x-minP.x)*curand_uniform(randState)+minP.x;
        ranP3.y=(maxP.y-minP.y)*curand_uniform(randState)+minP.y;
        ranP3.z=(maxP.z-minP.z)*curand_uniform(randState)+minP.z;
        return ranP3;
}

void initializeCodebook(point3 * codebook, point3 minPoint,
                        point3 maxPoint,int nClusters)
{
        //Initialize centroids via bounding box uniform sampling.
        printf("Initializing centroids\n");
        printf("Max point:\t"); printPoint3(maxPoint);
        printf("\nMin point:\t"); printPoint3(minPoint);
        printf("\n");
        srand(time(NULL));
        printf("--Initial centroids--\n" );
        for (unsigned int i = 0; i < nClusters; i++)
        {
                codebook[i] = randomPoint3(minPoint,maxPoint);
                printPoint3(codebook[i]);
        }
        return;
}

void initializeCodebook(point3 * codebook, point3* points,
                        int pointSize, int nClusters)
{
        //Initilaize centroids in codebook by sampling points in the dataset. named as inner method
        printf("Initializing centroids\n");
        srand(time(NULL));

        for (unsigned int i = 0; i < nClusters; i++)
        {
                int rndIdx = rand() % (pointSize-1);
                codebook[i]= points[rndIdx];
                //printPoint3(codebook[i]);
        }
        return;
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
        /*Computes the distance from points to centroids
        Legacy implmentation with makePartition
        The distance array is a 2d array but i flattened it into a one dimensional array cause i am
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
        /*Uses distance matrix to compute partition and histogram for centroids
        Legacy implementation*/
        int idx  = threadIdx.x + blockIdx.x * blockDim.x;
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
        return;
}


__global__ void separationKernel(point3 * points, point3 * centroids,
                                 int *histogram, int *partition,
                                 int k, int n)
{
        /*This kernerl computes the partition for all the points
        and the current centroids
        Input:
            points:     Points cloud
            centroids:  Previously assigned centroids
        Output
            histogram:  Vector of size k, each elements stores the count
                      of points that belong to a certain centroid
            partition:  Array of labels, assigns each point to a centroid
        */
        int idx = threadIdx.x + blockIdx.x * blockDim.x;
        if (idx<n)
        {
                float minDist = euclideanDistance(points[idx],centroids[0]);
                int minIxd = 0;
                for (int i =1; i < k; i++) {
                        float dist = euclideanDistance(points[idx],centroids[i]);
                        if (dist < minDist) {
                                minDist = dist;
                                minIxd = i;
                        }
                }
                partition[idx]= minIxd;
                atomicAdd(&histogram[minIxd],1);
        }
        return;
}

__global__ void prepareReduceArray(point3 *points, int *partition, point3 *reduceArrray, int centroid, int n)
{
        /*takes the whole partition array and outputs a single array that only has points belonging to a
        particular centroid 
        Input:
            points:      Point cloud
            partition:   Array of labels, one for each point corresponding to its closest centroid 
        Output:
            reduceArray: Array with only points belonging to "centroid" or the zero vector
                       This can be later reduced to compute the new centroid 
        */
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
                                     int n)
{
        /*Partial parallel reduction, computes the sum of all points up to block level
        Inputs:
            points:          Point cloud
        Output:
            partialReesult:   Reduced array if blocks>THREAD then this must be called
                             again to reduce block level result            */
        
        __shared__ point3 pointsShared[THREADS];
        int idx = blockIdx.x*blockDim.x+threadIdx.x;
        int tid = threadIdx.x;
        //This is the way to do it
        //If shared memory is no zeroed
        ///Sometimes i will read random stuff from out of range
        point3 zero;
        zero.x=0; zero.y=0; zero.z=0;
        pointsShared[tid]=zero;
        __syncthreads();

        if(idx < n)
        {
                //only copy from partition j
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
                if(tid==0) {
                        //printf("Block: %d\n",blockIdx.x);
                        //printf("Number of elements in  %d is %d\n",k, partitionShared[0]);
                        //partitionOut[blockIdx.x] = partitionShared[0];
                        // printf("Partial Sum:%f,%f,%f\n",
                        //        pointsShared[0].x,pointsShared[0].y,pointsShared[0].z);
                        partialResult[blockIdx.x] = pointsShared[0];

                }
        }
}

__global__ void recalcCentroidsOuter(point3 * points,
                                     point3* centroids,
                                     int *histogram,
                                     int k, int n,
                                     curandState *crs,
                                     point3 maxP,point3 minP
                                     )
{
        /*Computes the final reduce operation on points and computes
          new centroids:
        Input:
            points:    Point Cloud
            histogram: Histogram of points belonging to each centroid
        Output:
            centroids: Previous codebook
            k:         Numbre of centroids            */
        //histogram is a count of how many elements belong to each centroid
        __shared__ point3 pointsShared[THREADS];
        int idx = blockIdx.x*blockDim.x+threadIdx.x;
        int tid = threadIdx.x;
        point3 zero;
        zero.x=0; zero.y=0; zero.z=0;
        pointsShared[tid]=zero;
        __syncthreads();
        if(idx < n)
        {
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
                        if(histogram[k]>0) {
                                centroids[k] = mulPoint3(pointsShared[0],1.0/histogram[k]);
                        }
                        else
                        {
                                point3 rp3 = randomPoint3(crs,maxP,minP);
                                centroids[k] = rp3;
                                //addPoint3(centroids[k],rp3);
                        }
                }
        }

}

__global__ void recalcCentroidsOuter(point3 * points,
                                     point3* centroids,
                                     int *histogram,
                                     int k, int n
                                     )
{
        /*Computes the final reduce operation on points and computes
          new centroids:
        Input:
            points:    Point Cloud
            histogram: Histogram of points belonging to each centroid
        Output:
            centroids: Previous codebook
            k:         Numbre of centroids            */
        //histogram is a count of how many elements belong to each centroid
        __shared__ point3 pointsShared[THREADS];
        int idx = blockIdx.x*blockDim.x+threadIdx.x;
        int tid = threadIdx.x;
        point3 zero;
        zero.x=0; zero.y=0; zero.z=0;
        pointsShared[tid]=zero;
        __syncthreads();
        if(idx < n)
        {
                //set all to 0; 
                // setZerosDevice(pointsShared,THREADS); //Size of shared memory
                // __syncthreads();

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
                        if(histogram[k]>0) {
                                // printf("Centroid[%d] prior:%f,%f,%f\n",
                                //        k,centroids[k].x,centroids[k].y,centroids[k].z);
                                // printf("--Poinst[%d] posterior:%f,%f,%f\n",
                                //        0,pointsShared[0].x,pointsShared[0].y,pointsShared[0].z);
                                // printf("Centroid[%d] posterior:%f,%f,%f\n",
                                //        k,centroids[k].x,centroids[k].y,centroids[k].z);
                                centroids[k] = mulPoint3(pointsShared[0],1.0/histogram[k]);
                        }
                }
        }
        //__syncthreads();
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


//Non random resample
bool kmeans(point3 *h_points, int *h_partition,
            point3* h_codebook, int *h_histogram,
            int iterations, int clusters, int nPoints)
{

        printf("Received: %d\n",nPoints);

        //Pointers
        point3 *d_points, *d_codebook;
        int *d_partition,  *d_histogram;
        point3 *d_reduceArray, *d_partialReduce;
        //sizes
        int nPointsSize   = nPoints*sizeof(point3);
        int clustersSize  = clusters*sizeof(point3);
        int partitionSize = nPoints*sizeof(int);
        int histogramSize = clusters*sizeof(int);

        //h_distances = (float *) malloc(distanceSize);
        //h_partition = (int *) malloc(partitionSize);
        cudaMalloc((void**)&d_points,nPointsSize);
        cudaMalloc((void**)&d_codebook,clustersSize);
        cudaMalloc((void**)&d_partition,partitionSize);

        cudaMalloc((void**)&d_reduceArray,nPointsSize);
        cudaMalloc((void**)&d_partialReduce,nPointsSize);
        cudaMalloc((void**)&d_histogram,histogramSize);

        cudaMemcpy(d_points,h_points,nPointsSize,cudaMemcpyHostToDevice);
        cudaMemcpy(d_codebook,h_codebook,clustersSize,cudaMemcpyHostToDevice);

        //int blks = nPoints/ THREADS;
        int blks = (nPoints + THREADS - 1) / THREADS;
        printf("Issuing %d blocks with %d threads\n",blks, THREADS);
        for (int m = 0; m <iterations; m++) {
                ///if blks > THREADS return error not enough memory
                cudaMemset(d_histogram, 0, histogramSize);
                blks = (nPoints + THREADS - 1) / THREADS;
                separationKernel<<<blks,THREADS>>>
                (d_points,d_codebook,d_histogram,d_partition,clusters,nPoints);
                for(int i= 0; i<clusters; i++)
                {
                        blks = (nPoints + THREADS - 1) / THREADS;
                        prepareReduceArray<<<blks,THREADS>>>
                        (d_points,d_partition,d_reduceArray,i,nPoints);
                        //printf(">>>>>First RUN on cluster [%d]\n",i );
                        recalcCentroidsInner<<<blks,THREADS>>>
                        (d_reduceArray,d_reduceArray,nPoints);
                        while (blks>THREADS) {
                                //In case we have more points than THREADS
                                //This accumulates block level result
                                int n = blks;
                                blks = (blks + THREADS - 1) / THREADS;
                                recalcCentroidsInner<<<blks,THREADS>>>
                                (d_reduceArray,d_reduceArray,n);
                                //cudaDeviceSynchronize();
                        }
                        //Aqui acabaria el while
                        recalcCentroidsOuter<<<1,THREADS>>>
                        (d_reduceArray,d_codebook,d_histogram,i,blks);
                }
        }
        cudaDeviceSynchronize();
        cudaMemcpy(h_partition,d_partition,
                   partitionSize,cudaMemcpyDeviceToHost);
        cudaMemcpy(h_codebook,d_codebook,
                   clustersSize,cudaMemcpyDeviceToHost);
        cudaMemcpy(h_partition,d_partition,
                   partitionSize,cudaMemcpyDeviceToHost);
        cudaMemcpy(h_histogram,d_histogram,
                   histogramSize,cudaMemcpyDeviceToHost);

        //printVec(h_partition,nPoints);
        //printf("---Optimized centroids---\n");
        //printPoint3Array(h_codebook, clusters);
        cudaFree(d_points); cudaFree(d_codebook);
        cudaFree(d_reduceArray); cudaFree(d_partialReduce); cudaFree(d_histogram);

        //free(h_distances);
        return true;
}

//random resample, if partion has no elements sample new centroid
bool kmeans(point3 *h_points, int *h_partition,
            point3* h_codebook, int *h_histogram,
            int iterations, int clusters, int nPoints,
            point3 maxP, point3 minP)
{

        printf("Received: %d\n",nPoints);

        //Pointers
        point3 *d_points, *d_codebook;
        int *d_partition,  *d_histogram;
        point3 *d_reduceArray, *d_partialReduce;
        //sizes
        int nPointsSize   = nPoints*sizeof(point3);
        int clustersSize  = clusters*sizeof(point3);
        int partitionSize = nPoints*sizeof(int);
        int histogramSize = clusters*sizeof(int);


        //h_partition = (int *) malloc(partitionSize);

        cudaMalloc((void**)&d_points,nPointsSize);
        cudaMalloc((void**)&d_codebook,clustersSize);
        cudaMalloc((void**)&d_partition,partitionSize);

        cudaMalloc((void**)&d_reduceArray,nPointsSize);
        cudaMalloc((void**)&d_partialReduce,nPointsSize);
        cudaMalloc((void**)&d_histogram,histogramSize);

        cudaMemcpy(d_points,h_points,nPointsSize,cudaMemcpyHostToDevice);
        cudaMemcpy(d_codebook,h_codebook,clustersSize,cudaMemcpyHostToDevice);

        //curand stuff
        curandState *d_state;
        cudaMalloc(&d_state, sizeof(curandState));
        setup_kernel<<<1,1>>>(d_state);

        //int blks = nPoints/ THREADS;
        int blks = (nPoints + THREADS - 1) / THREADS;
        printf("Issuing %d blocks with %d threads\n",blks, THREADS);
        for (int m = 0; m <iterations; m++) {
                ///if blks > THREADS return error not enough kenerls
                blks = (nPoints + THREADS - 1) / THREADS;
                cudaMemset(d_histogram, 0, histogramSize);
                blks = (nPoints + THREADS - 1) / THREADS;
                separationKernel<<<blks,THREADS>>>
                (d_points,d_codebook,d_histogram,d_partition,clusters,nPoints);
                for(int i= 0; i<clusters; i++)
                {
                        blks = (nPoints + THREADS - 1) / THREADS;
                        prepareReduceArray<<<blks,THREADS>>>
                        (d_points,d_partition,d_reduceArray,i,nPoints);
                        //printf(">>>>>First RUN on cluster [%d]\n",i );
                        recalcCentroidsInner<<<blks,THREADS>>>
                        (d_reduceArray,d_reduceArray,nPoints);
                        while (blks>THREADS) {
                                int n = blks;
                                blks = (blks + THREADS - 1) / THREADS;
                                recalcCentroidsInner<<<blks,THREADS>>>
                                (d_reduceArray,d_reduceArray,n);

                        }
                        recalcCentroidsOuter<<<1,THREADS>>>
                        (d_reduceArray,d_codebook,d_histogram,i,blks,d_state,maxP,minP);
                }
        }
        cudaDeviceSynchronize();
        cudaMemcpy(h_partition,d_partition,
                   partitionSize,cudaMemcpyDeviceToHost);
        cudaMemcpy(h_codebook,d_codebook,
                   clustersSize,cudaMemcpyDeviceToHost);
        cudaMemcpy(h_partition,d_partition,
                   partitionSize,cudaMemcpyDeviceToHost);
        cudaMemcpy(h_histogram,d_histogram,
                   histogramSize,cudaMemcpyDeviceToHost);

        //printVec(h_distances,nPoints*clusters);
        //printVec(h_partition,nPoints);
        printf("---Optimized centroids---\n");
        printPoint3Array(h_codebook, clusters);
        cudaFree(d_points);  cudaFree(d_codebook);
        cudaFree(d_reduceArray); cudaFree(d_partialReduce); cudaFree(d_histogram);
        cudaFree(d_state);

        return true;
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


void recalcCentroids(point3 *codebook, point3 *points,
                     int* partition, int* histogram,
                     int nPoints, int clusters)
{
        //recalc centroids and make histogram
        for (int i = 0; i < clusters; i++) {
                codebook[i].x=0;
                codebook[i].y=0;
                codebook[i].z=0;
                //  printf("Cluster[%d]: %d\n",i,histogram[i] );
        }
        for (int i = 0; i < nPoints; i++)
        {
                codebook[partition[i]].x+=points[i].x;
                codebook[partition[i]].y+=points[i].y;
                codebook[partition[i]].z+=points[i].z;

        }
        for (int i = 0; i < clusters; i++) {
                if (histogram[i]>0) {
                        codebook[i]=mulPoint3(codebook[i],1.0/histogram[i]);
                }
                // printf("New Codebook @ [%d] with %d points\n",i,histogram[i]);
                // printPoint3(codebook[i]);
        }
        return;
}
