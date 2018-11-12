#include "adjacency_graph.h"

adjacencyGraph::adjacencyGraph(ros::NodeHandle &nh)
{
        nh_=nh;
        ros::NodeHandle nh_priv("~");
        nh_priv.param<float>("free_thr",freeThr,0.1);
        nh_priv.param<float>("safety_height",safetyHeight,1.2);
        nh_priv.param<float>("safety_radius",safetyRadius,0.75);
        nh_priv.param<float>("connection_radius",connectionRadius,0.4);
        nh_priv.param<int>("k_neighboors",kNeighboors,6);
        nh_priv.param<std::string>("graph_file",graphFile,"adjGraph.txt");
        nh_priv.param<float>("max_dist",maxDist,2); //max distance between neighboors

        quantSub = nh_.subscribe("codebook",10, &adjacencyGraph::codebookCallback, this);
        graphMakeSub = nh_.subscribe("make_graph",1, &adjacencyGraph::makeGraph,this);
        graphClearSub = nh_.subscribe("clear_graph",1, &adjacencyGraph::clearGraph,this);
        markerPub =
                nh_.advertise<visualization_msgs::Marker>("graph_marker",2);
        centroidsMarkerPub=
                nh_.advertise<visualization_msgs::Marker>("centroids_marker",2);
        std::cout << "Starting Adjacency graph maker node by CoyoSoft" << '\n';
        std::cout << graphFile << '\n';

}

adjacencyGraph::~adjacencyGraph()
{

}

void adjacencyGraph::clearGraph(const std_msgs::Empty &msg)
{
        std::cout << "!!!Clearing all codebooks!!!!" << '\n';
        occCodebook.clear();
        freeCodebook.clear();
        makeCentroidsMarkerAndPublish(freeCodebook,true);
}

void adjacencyGraph::makeCentroidsMarkerAndPublish( pointArray &codebook, bool free)
{
        const float radius = 0.05;
        visualization_msgs::Marker centroidsMarker;
        //centroidsMarker.points.resize(codebook.size());
        centroidsMarker.ns = "centroids";
        centroidsMarker.action = visualization_msgs::Marker::ADD;
        centroidsMarker.header.frame_id = cloudFrame;
        centroidsMarker.header.stamp = ros::Time();
        centroidsMarker.type = visualization_msgs::Marker::CUBE_LIST;
        centroidsMarker.pose.orientation.w = 1.0;
        centroidsMarker.scale.x = radius;
        centroidsMarker.scale.y = radius;
        centroidsMarker.scale.z = radius;

        if (free) {
                centroidsMarker.id = 1;
                centroidsMarker.color.r=0.5;
                centroidsMarker.color.b=0.5;
                centroidsMarker.color.g=1.0;
                centroidsMarker.color.a=1.0;

        }
        else
        {
                centroidsMarker.id = 2;
                centroidsMarker.color.r=1.0;
                centroidsMarker.color.b=0.0;
                centroidsMarker.color.g=0.25;
                centroidsMarker.color.a=1.0;
        }
        centroidsMarker.points = codebook;
        centroidsMarkerPub.publish(centroidsMarker);
        return;
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
        cloudFrame = msg.header.frame_id;
        stamp = msg.header.stamp;
        //TODO use remove erase idiom

        pointArray centroids = msg.centroids;
        for (int i = 0; i < centroids.size(); i++)
        {
                pointGeom cnt = centroids[i];
                if (cnt.z>=freeThr)
                {
                        occCodebook.push_back(cnt);
                        //centroids.erase(centroids.begin()+i);
                }
        }
        for (int i = 0; i < centroids.size(); i++)
        {
                pointGeom cnt = centroids[i];
                if (cnt.z<freeThr) {
                        if (!validateFreeCentroid(cnt,safetyHeight,safetyRadius))
                        {
                                freeCodebook.push_back(cnt);
                        }
                        // else
                        // {
                        //         occCodebook.push_back(cnt);
                        // }
                }
        }

        edges = freeCodebook.size()+occCodebook.size();
        std::cout << "Got a codebook of:" << centroids.size() << " points"<< '\n';
        std::cout << "In frame " << cloudFrame << " on Stamp " << stamp << "\n";
        std::cout << "Global map is " << freeCodebook.size() << "points long\n";
        std::cout << "Occupied space is " << occCodebook.size()<< "points long\n";

        makeCentroidsMarkerAndPublish(freeCodebook,true);
        makeCentroidsMarkerAndPublish(occCodebook,false);

        return;
}

void adjacencyGraph::makeGraph(const std_msgs::Empty &msg)
{
        //Allocate an initialize adj matrix
        std::cout << "Adj Graph->Constructing adjacency graph" << '\n';
        if (freeCodebook.empty()) {
                std::cout << "Adj Graph-> Codebook is empty" << '\n';
                return;
        }
        adjacencyList adjList(freeCodebook.size());
        std::cout << "**********" << '\n';
        Knn(freeCodebook,adjList);
        //Save to disk as file
        printAdjacencyList(adjList);
        saveAdjGraph(graphFile,adjList);
        makeVizMsgAndPublish(adjList);
        return;
}

pointGeom adjacencyGraph::makeGeometryMsg(float x, float y, float z)
{
        //Helper function, to go from x,y,z to geometry_msgs::Point
        pointGeom p;
        p.x=x,p.y=y,p.z=z;
        return p;
}

bool adjacencyGraph::validateFreeCentroid(pointGeom &freeCentroid, float height, float radius)
{
        //Checks if the free point is actually far enoguh from every occupied
        //centroid
        //Basically collision with the occupied codebook with  a cylinder
        //with the robot height and radios of robot footprint as well and
        //Centered on free centroid
        bool collision = false;
        pointGeom vz = makeGeometryMsg(0,0,1);
        for (size_t i = 0; i < occCodebook.size(); i++)
        {
                collision=cilynderCollision(freeCentroid,vz, occCodebook[i],height,radius);
                if (collision)
                {
                        // printf("Collision with : ");
                        // printf("[%f,%f,%f] &",
                        //        freeCentroid.x,freeCentroid.y,freeCentroid.z );
                        // printf(" [%f,%f,%f]\n",
                        //        occCodebook[i].x,occCodebook[i].y,occCodebook[i].z );
                        break;
                }
        }
        return collision;
}

bool adjacencyGraph::validateConnection(pointGeom p1, pointGeom p2, float radius)
{
        //Chceck if a connection between two free centroids is intersected by an occupied
        //point
        Eigen::Vector3d p1Eigen(p1.x,p1.y,p1.z);
        Eigen::Vector3d p2Eigen(p2.x,p2.y,p2.z);
        //std::cout << p1Eigen << "<---->"<< p2Eigen<< '\n';
        bool collision = false;
        for (size_t i = 0; i < occCodebook.size(); i++) {
                Eigen::Vector3d q(occCodebook[i].x,occCodebook[i].y,occCodebook[i].z);
                collision = cylinderCollision(p1Eigen,p2Eigen,q,radius);
                if (collision) {
                        break;
                }
        }
        return collision;
}


bool adjacencyGraph::cylinderCollision(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d q, float radius)
{
        Eigen::Vector3d p2_p1=p2-p1;
        // std::cout << "Conecttions from " << '\n';
        // std::cout << "p1: " <<  p1<<'\n';
        // std::cout << "p2: " <<  p2<<'\n';
        // std::cout << "q: " <<  q<<'\n';
        if (p2_p1.norm()<0.001) {
                std::cout << "Same point no cylinder " <<  p2_p1.norm() <<'\n';
                return true;
        }
        float distanceLid1 = (q-p1).dot(p2_p1);
        bool collision=false;
        if (distanceLid1>=0)
        {
                float distanceLid2=(q-p2).dot(p1-p2);
                if (distanceLid2>=0)
                {
                        //double num = (q-p1).cross(p2_p1).norm();
                        double num = (q-p1).cross(q-p2).norm();
                        double den = p2_p1.norm();
                        float distanceSpine = num/den;

                        if (distanceSpine<radius)
                        {
                                //printf("Connection collision distance = %f\n", distanceSpine  );
                                collision=true;
                        }
                        //printf("Query distance = %f\n", distanceSpine  );
                }

        }
        return collision;
}

bool adjacencyGraph::cilynderCollision(pointGeom pi,pointGeom v, pointGeom q, float height, float radius)
{
        //checkl if point collides with cylinder
        Eigen::Vector3d p1(pi.x,pi.y,pi.z);
        Eigen::Vector3d dir(v.x,v.y,v.z);
        dir=dir.normalized();
        Eigen::Vector3d p2 = p1+height*dir;
        Eigen::Vector3d testPoint(q.x,q.y,q.z);
        Eigen::Vector3d p2_p1 = p2-p1;
        //chcek distance to cylinder lids. If >= 0 the point is inside
        //The cilinder
        float distanceLid1=(testPoint-p1).dot(p2_p1);
        if (distanceLid1<0) {
                return false;
        }
        float distanceLid2=(testPoint-p2).dot(p1-p2);
        if (distanceLid2<0) {
                return false;
        }
        //check if inside cylinder radius
        double num = (testPoint-p1).cross(p2_p1).norm();
        double den = p2_p1.norm();
        float distanceSpine = num/den;
        if (distanceSpine>radius) {
                return false;
        }
        //printf("Collision distance = %f\n", distanceSpine  );
        return true;
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
        connections.points.clear();
        for (size_t i = 0; i < l.size(); i++)
        {
                for (size_t j = 0; j < l[i].size(); j++)
                {
                        //We can acces adjList as:
                        //l[i][j] = k meaning node i is connected to k
                        //j is an iterator
                        int k = l[i][j];
                        connections.points.push_back(freeCodebook[i]);
                        connections.points.push_back(freeCodebook[k]);
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

void adjacencyGraph::printCentroids(pointArray &centroids)
{
        std::cout << "**********" << '\n';
        for (size_t i = 0; i < freeCodebook.size(); i++) {
                std::cout << freeCodebook[i].x << ","
                          << freeCodebook[i].y << ","
                          << freeCodebook[i].z << '\n';
        }
        std::cout << "**********" << '\n';
}

void adjacencyGraph::printPoint(pointGeom &p)
{

        std::cout <<"["<< p.x << ","
                  << p.y << ","
                  << p.z << "]\n";

}

void adjacencyGraph::Knn(pointArray &centroids, adjacencyList & adjL)
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
                        //printf("[%d,%f],", distances[k].label,distances[k].dist);
                        if(distances[k].dist <= maxDist)
                        {
                                int label=distances[k].label; //wich centroid
                                // printCentroids(centroids);
                                // printf("C[%d]",label );                        printPoint(centroids[label]);
                                // printf("C[%d]",i );                        printPoint(centroids[i]);

                                if (!validateConnection(centroids[label],centroids[i],
                                                        connectionRadius))
                                {
                                        adjL[i].push_back(label);
                                }
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
                if (l[i].size()<1) {
                        continue;
                }
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


void adjacencyGraph::saveAdjGraph(std::string filename, adjacencyList &adjL)
{
        //Saves the adjacency matrix to a file to disk.
        std::cout << "Writing to "<<filename<<"\n";
        std::ofstream graphOut;
        int freeNodes =freeCodebook.size(); int occNodes =  occCodebook.size();
        int totalNodes = freeNodes+occNodes;
        graphOut.open(filename.c_str());
        graphOut<<"Clusters: " << totalNodes << "\n";
        graphOut<<"Codebook: x,y,z,label\n";
        graphOut << "Occupied Nodes: " << occNodes <<"\n";
        for(int i=0; i<occCodebook.size(); i++)
        {
                graphOut<<occCodebook[i].x<<",";
                graphOut<<occCodebook[i].y<<",";
                graphOut<<occCodebook[i].z<<",";
                graphOut<<i<<"\n";
        }
        graphOut << "Free Nodes:" << freeNodes<<"\n";
        for(int i=0; i<freeCodebook.size(); i++)
        {
                graphOut<<freeCodebook[i].x<<",";
                graphOut<<freeCodebook[i].y<<",";
                graphOut<<freeCodebook[i].z<<",";
                graphOut<<i<<"\n";
        }
        graphOut<<"Adjacency List:\n";
        //Size is theoretically the same as edges
        //Or number of nodes
        //Node i: J K L etc
        for(int i=0; i<adjL.size(); i++)
        {
                // if (adjL[i].size()<1) {
                //         //If node is not connected to anything why even bother
                //         //Saving it to the graph?
                //         continue;
                // } //Saved to make graph continous
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
