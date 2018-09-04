#include "adjacency_graph.h"

adjacencyGraph::adjacencyGraph(ros::NodeHandle &nh)
{
        nh_=nh;
        ros::NodeHandle nh_priv("~");
        quantSub = nh_.subscribe("quantized_space",10, &adjacencyGraph::quantizedCallback, this);
        graphMakeSub = nh_.subscribe("make_graph",1, &adjacencyGraph::makeGraph,this);
        nh_priv.param<int>("k_neighboors",kNeighboors,3);
        nh_priv.param<std::string>("graph_file",graphFile,"adjGraph.txt");
        nh_priv.param<float>("max_dist",maxDist,3); //max distance between neighboors
        std::cout << "Starting Adjacency graph maker node by CoyoSoft" << '\n';
        std::cout << graphFile << '\n';

}

adjacencyGraph::~adjacencyGraph()
{
        if (adjMat[0]!=NULL) {
                for (size_t i = 0; i < edges; i++) {
                        free(adjMat[i]);
                }
                free(adjMat);
        }
}

void adjacencyGraph::quantizedCallback (const space_quantization::quantizedSpace &msg)
{
        //unpacking
        sensor_msgs::PointCloud2 space = msg.space;
        int n=space.height*space.width;
        std::cout << "Got label cloud of:" << n << "Points"<< '\n';
        cloudFrame = space.header.frame_id;
        stamp = space.header.stamp;
        //Codebook can be requested asynchronously
        std::cout << "In frame " << cloudFrame << " on Stamp " <<stamp<< '\n';
        codebook = msg.codebook;
        edges = codebook.size();
        //TODO cuda function to validate all points.
        //They must not be near any occ space point by less than somr threshold
        //else the robot will crash
        //validateCodebook(codebook);
}

void adjacencyGraph::makeGraph(const std_msgs::Empty &msg)
{
        //Allocate an initialize adj matrix
        if (codebook.empty()) {
                return;
        }
        float **adjG =makeAdjacencyMat(edges);
        Knn(codebook,adjG);
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

float ** adjacencyGraph::makeAdjacencyMat(int nEdges)
{
        //Allocate space for adjacency matrix
        //and set all to -1 (not connected)
        float **adjM;
        adjM=(float **)malloc(nEdges*sizeof(float*));
        for (int i = 0; i < nEdges; i++)
        {
                adjM[i] = (float *)malloc(nEdges*sizeof(float));
        }

        for (int i = 0; i < nEdges; i++) {
                for (int j = 0; j < nEdges; j++) {
                        adjM[i][j]=-1;
                }
        }
        return adjM;
}

int ** adjacencyGraph::makeAdjacencyList(int nEdges, int k)
{
        //Allocate space for adjacency list
        //n nodes with k neighboors each
        int **adjL;
        adjL=(int **)malloc(nEdges*sizeof(int*));
        for (int i = 0; i < nEdges; i++)
        {
                adjL[i] = (int *)malloc(k*sizeof(int));
        }
        return adjL;
}


void adjacencyGraph::printAdjacencyMat(float ** adjM, int n)
{
        //as the name implies  check it is alive!
        //Mostly for debuggin
        printf("\t");
        for (size_t i = 0; i < n; i++) {
                printf("%ld,", i );
        }
        printf("\n");
        for (size_t i = 0; i < n; i++)
        {
                printf("%ld\t", i );
                for (size_t j = 0; j < n; j++)
                {
                        printf("%.3f\t",adjM[i][j]);
                }
                printf("\n");
        }
}

void adjacencyGraph::saveAdjGraph(std::string filename, pointArray centroids, float ** adjG)
{
        //Saves the adjacency matrix to a file to disk.
        std::cout << "Writing to "<<filename<<"\n";
        std::ofstream graphOut;
        graphOut.open(filename.c_str());
        graphOut<<"Clusters: " <<edges  << "\n";
        graphOut<<"Codebook: x,y,z,label\n";
        for(int i=0; i<edges; i++)
        {
                graphOut<<centroids[i].x<<",";
                graphOut<<centroids[i].y<<",";
                graphOut<<centroids[i].z<<",";
                graphOut<<i<<"\n";
        }
        graphOut<<"Adjacency Matrix:\n";
        for(int i=0; i<edges; i++)
        {
                for (int j = 0; j < edges; j++) {
                        graphOut<<adjG[i][j]<<",";

                }
                graphOut<<"\n";
        }
        graphOut.close();
        return;
}

float adjacencyGraph::distance(geometry_msgs::Point p1,geometry_msgs::Point p2)
{
        geometry_msgs::Point p1p2;
        p1p2.x=p1.x-p2.x;
        p1p2.y=p1.y-p2.y;
        p1p2.z=p1.z-p2.z;
        return sqrtf(p1p2.x*p1p2.x+p1p2.y*p1p2.y+p1p2.z*p1p2.z);
}

float adjacencyGraph::norm(geometry_msgs::Point dp)
{
        return sqrtf(dp.x*dp.x+dp.y*dp.y+dp.z*dp.z);
}

bool adjacencyGraph::compareDistance(distanceLabel i, distanceLabel j)
{
        return i.dist<j.dist;
}

void adjacencyGraph::Knn(pointArray centroids, float ** adjG)
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
                for (int k = 1; k < 4; k++) {
                        // printf("[%d,%f],", distances[k].label,distances[k].dist);
                        adjG[i][distances[k].label]=distances[k].dist;
                }

        }
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
