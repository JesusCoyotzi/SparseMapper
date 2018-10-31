#include "adjacency_graph.h"

adjacencyGraph::adjacencyGraph(ros::NodeHandle &nh)
{
        nh_=nh;
        ros::NodeHandle nh_priv("~");
        voxelSub = nh_.subscribe("voxelized_space",10, &adjacencyGraph::codebookCallback, this);
        quantSub = nh_.subscribe("quantized_space",10, &adjacencyGraph::quantizedCallback, this);
        graphMakeSub = nh_.subscribe("make_graph",1, &adjacencyGraph::makeGraph,this);
        markerPub = nh_.advertise<visualization_msgs::Marker>("graph_marker",2);
        nh_priv.param<int>("k_neighboors",kNeighboors,6);
        nh_priv.param<std::string>("graph_file",graphFile,"adjGraph.txt");
        nh_priv.param<float>("max_dist",maxDist,2); //max distance between neighboors
        std::cout << "Starting Adjacency graph maker node by CoyoSoft" << '\n';
        std::cout << graphFile << '\n';

}

adjacencyGraph::~adjacencyGraph()
{

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

void adjacencyGraph::codebookCallback(const space_quantization::codebook &msg)
{
        codebook = msg.centroids;
        edges = codebook.size();
        cloudFrame = msg.header.frame_id;
        stamp = msg.header.stamp;
        std::cout << "Got a codebook of:" << edges << " points"<< '\n';
        std::cout << "In frame " << cloudFrame << " on Stamp " << stamp << "\n";
        //ideally will validate the codebook points
        return;
}

void adjacencyGraph::makeGraph(const std_msgs::Empty &msg)
{
        //Allocate an initialize adj matrix
        std::cout << "Adj Graph->Constructing adjacency graph" << '\n';
        if (codebook.empty()) {
                std::cout << "Adj Graph-> Codebook is empty" << '\n';
                return;
        }
        adjacencyList adjList(edges);
        Knn(codebook,adjList);
        //printAdjacencyMat(adjG,edges);
        //Save to disk as file
        printAdjacencyList(adjList);
        saveAdjGraph(graphFile,codebook,adjList);
        makeVizMsgAndPublish(adjList);
        return;
}

void adjacencyGraph::makeVizMsgAndPublish(adjacencyList l)
{
        visualization_msgs::Marker connections;
        connections.header.frame_id=cloudFrame;
        connections.header.stamp=ros::Time::now();
        connections.ns = "grafo";
        connections.pose.orientation.w = 1.0;
        connections.id = 1;
        connections.type = visualization_msgs::Marker::LINE_LIST;
        connections.scale.x = 0.05;
        connections.color.r=1.0;
        connections.color.b=1.0;
        connections.color.g=1.0;
        connections.color.a=1.0;
        for (size_t i = 0; i < l.size(); i++)
        {
                for (size_t j = 0; j < l[i].size(); j++)
                {
                        //We can acces adjList as:
                        //l[i][j] = k meaning node i is connected to k
                        //j is an iterator
                        int k = l[i][j];
                        connections.points.push_back(codebook[i]);
                        connections.points.push_back(codebook[k]);
                }
        }
        markerPub.publish(connections);
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

void adjacencyGraph::Knn(pointArray centroids, adjacencyList & adjL)
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
                //Fisrt element is always the node compared
                //with itself, distance=0 by definition
                for (int k = 1; k < kNeighboors; k++) {
                        // printf("[%d,%f],", distances[k].label,distances[k].dist);
                        if(distances[k].dist <= maxDist)
                        {
                                adjL[i].push_back(distances[k].label);
                        }
                }

        }
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

void adjacencyGraph::printAdjacencyList(adjacencyList l)
{
        //as the name implies  check it is alive!
        //Mostly for debuggin
        printf("\n");
        for (size_t i = 0; i < l.size(); i++)
        {
                printf("%ld:\t", i );
                for (size_t j = 0; j < l[i].size(); j++)
                {
                        printf("%d,",l[i][j]);
                }
                printf("\n");
        }
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

void adjacencyGraph::saveAdjGraph(std::string filename, pointArray centroids, adjacencyList &adjL)
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
        graphOut<<"Adjacency List:\n";
        //Size is theoretically the same as edges
        //Or number of nodes
        //Node i: J K L etc
        for(int i=0; i<adjL.size(); i++)
        {
                graphOut<<i<<":";
                for (int j = 0; j < adjL[i].size(); j++) {
                        graphOut<<adjL[i][j]<<",";

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
