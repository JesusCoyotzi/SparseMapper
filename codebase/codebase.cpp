bool adjacencyMap::validateNode(pointGeom p1)
{
        //Chceck if a point is too close to occupied node
        //REturns true if collision exists
        Eigen::Vector3d p1Eigen(p1.x,p1.y,p1.z);
        Eigen::Vector3d p2Eigen(p1.x,p1.y,p1.z+safetyHeight);
        //std::cout << p1Eigen << "<---->"<< p2Eigen<< '\n';
        std::vector<distanceLabel> distances(occupiedNodes.size());
        for (int j = 0; j < occupiedNodes.size(); j++)
        {
                Eigen::Vector3d q(occupiedNodes[j].x,occupiedNodes[j].y,occupiedNodes[j].z);
                distances[j].dist=(p1Eigen-q).norm();
                distances[j].label=j;
        }
        std::sort(distances.begin(),distances.end());

        bool collision = false;
        for (size_t i = 0; i < distances.size(); i++) {
                if (distances[i].dist>(safetyRadius+safetyHeight)) {
                        //They are ordered there wont be any more centroids closer to safetySphere
                        break;
                }
                int label = distances[i].label;
                Eigen::Vector3d q(occupiedNodes[label].x,occupiedNodes[label].y,occupiedNodes[label].z);
                //Check if it is worth doing cilinder collision
                //If node is closer than safetyH + safetyRadius check else ignore
                collision = cylinderCollision(p1Eigen,p2Eigen,q,safetyRadius);
                if (collision) {
                        break;
                }

        }
        return collision;
}

void adjacencyGraph::makeGraph(const std_msgs::Empty &msg)
{
        //Allocate an initialize adj matrix
        if (codebook.empty()) {
                return;
        }
        float ** 0 makeAdjacencyMat(edges);
        Knn(codebook,adjList);
        //printAdjacencyMat(adjG,edges);
        //Save to disk as file
        saveAdjGraph(graphFile,codebook,adjG);
        //free memory
        for (size_t i = 0; i < edges; i++) {
                free(adjG[i]);
        }
        free(adjG);
        return;
}


void adjacencyGraph::Knn(pointArray centroids, int ** adjG)
{
        //calculate the k nearest neighborrs of the centroids
        //And return the adjacency graph
        int nCnt = centroids.size();
        std::vector<distanceLabel> distances(nCnt);
        for (int i = 0; i <nCnt; i++)
        {
                for (int j = 0; j < nCnt; j++)
                {
                        distances[j].dist=distance(centroids[i],centroids[j]);
                        distances[j].label=j;
                }
                std::sort(distances.begin(),distances.end(),compareDistance);
                for (int k = 0; k < 3; k++) {
                        // printf("[%d,%f],", distances[k].label,distances[k].dist);
                        adjG[i][k]=distances[k].label;
                }
                std::cout << '\n';
        }
}


#include <stdio.h>

//from::https://stackoverflow.com/questions/34596490/cuda-reduce-algorithm
/* -------- KERNEL -------- */
__global__ void reduce_kernel(float * d_out, float * d_in, const int size)
{
  // position and threadId
  int pos = blockIdx.x * blockDim.x + threadIdx.x;
  int tid = threadIdx.x;

  // do reduction in global memory
  for (unsigned int s = blockDim.x / 2; s>0; s>>=1)
  {
    if (tid < s)
    {
      if (pos+s < size) // Handling out of bounds
      {
        d_in[pos] = d_in[pos] + d_in[pos+s];
      }
    }
    __syncthreads();
  }

  // only thread 0 writes result, as thread
  if ((tid==0) && (pos < size))
  {
    d_out[blockIdx.x] = d_in[pos];
  }
}

/* -------- KERNEL WRAPPER -------- */
void reduce(float * d_out, float * d_in, int size, int num_threads)
{
  // setting up blocks and intermediate result holder
  int num_blocks = ((size) / num_threads) + 1;
  float * d_intermediate;
  cudaMalloc(&d_intermediate, sizeof(float)*num_blocks);
  cudaMemset(d_intermediate, 0, sizeof(float)*num_blocks);
  int prev_num_blocks;
  // recursively solving, will run approximately log base num_threads times.
  do
  {
    reduce_kernel<<<num_blocks, num_threads>>>(d_intermediate, d_in, size);

    // updating input to intermediate
    cudaMemcpy(d_in, d_intermediate, sizeof(float)*num_blocks, cudaMemcpyDeviceToDevice);

    // Updating num_blocks to reflect how many blocks we now want to compute on
      prev_num_blocks = num_blocks;
      num_blocks = num_blocks / num_threads + 1;

    // updating intermediate
    cudaFree(d_intermediate);
    cudaMalloc(&d_intermediate, sizeof(float)*num_blocks);
    size = num_blocks*num_threads;
  }
  while(num_blocks > num_threads); // if it is too small, compute rest.

  // computing rest
  reduce_kernel<<<1, prev_num_blocks>>>(d_out, d_in, prev_num_blocks);

}
