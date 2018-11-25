#include "sparse_map_server/adjacencyMap.h"

adjacencyMap::adjacencyMap()
{
        //default empty constructor
        return;
}


adjacencyMap::adjacencyMap(std::string mapFileName)
{
        loadMap(mapFileName);
        return;
}

adjacencyMap::adjacencyMap(std::string mapFile,
                           float safeHeight,float safeRadius,
                           float cRadius, float mxDist, float minDist, int kNeighboors)
{
        //Setter for params
        this->safetyHeight = safeHeight;
        this->safetyRadius = safeRadius;
        this->connectionRadius = cRadius;
        this->maxDist = mxDist;
        this->minDist = minDist;
        this->kNeighboors = kNeighboors;
        std::cout << "Parameters set for adj map:" << '\n';
        std::cout << "Safety Heigth: " << safetyHeight  << '\n';
        std::cout << "Safety Radius: " << safetyRadius  << '\n';
        std::cout << "Connection Radius: " << connectionRadius  << '\n';
        std::cout << "Max distance: " << maxDist  << '\n';
        std::cout << "Min distance: " << minDist  << '\n';
        std::cout << "K neighbors: " << this->kNeighboors  << '\n';

        loadMap(mapFile);
        return;
}


void adjacencyMap::setParams(float safeHeight,float safeRadius,
                             float cRadius, float mxDist, float minDist, int kNeighboors)
{
        //Setter for params
        this->safetyHeight = safeHeight;
        this->safetyRadius = safeRadius;
        this->connectionRadius = cRadius;
        this->maxDist = mxDist;
        this->minDist = minDist;
        this->kNeighboors = kNeighboors;
        return;
}

bool adjacencyMap::loadMap(std::string filename)
{
        std::cout << "Loading map from" << filename <<'\n';
        std::ifstream file(filename.c_str());
        if(!file.is_open())
        {
                std::cout << "Error reading file" << '\n';
                return false;
        }
        std::string line, tmp;
        std::getline(file,line);
        std::istringstream iss(line);
        int clusters,nOcc,nFree;
        //Chcek if top of file is indeed a graph file
        //Find number of clusters
        std::getline(iss, tmp, ':');
        iss>>clusters;
        std::cout << "Found: " << clusters<< " clusters \n";
        //Skip info headers
        std::getline(file,line);
        //Number of occupied nodes
        std::getline(file,line);
        iss.clear();
        iss.str(std::string());
        iss.str(line);
        std::getline(iss, tmp, ':');
        iss>>nOcc;
        std::cout << "Found " <<nOcc<< " occupied centroids" <<'\n';
        //Occupied nodes
        for (int i = 0; i < nOcc; i++)
        {
                pointGeom code;
                std::getline(file,line);
                code = parseCodeLine(line);
                occupiedNodes.push_back(code);

        }

        //Number of free nodes
        std::getline(file,line);
        iss.clear();
        iss.str(std::string());
        iss.str(line);
        std::getline(iss, tmp, ':');
        iss>>nFree;
        std::cout << "Found " <<nFree<< " free centroids" <<'\n';
        //Free nodes
        pointArray tempFree;
        for (int i = 0; i < nFree; i++)
        {
                pointGeom code;
                std::getline(file,line);
                code = parseCodeLine(line);
                if (!pruneNode(code,tempFree)) {
                        tempFree.push_back(code);
                }

        }
        std::cout << "Pruned to: " << tempFree.size() <<" non redundant nodes\n";
        for (size_t i = 0; i < tempFree.size(); i++) {
                bool isValidNode = !validateNode(tempFree[i]);
                if (isValidNode)
                {
                        freeNodes.push_back(tempFree[i]);
                }
        }

        std::cout << "Only " << freeNodes.size() <<" valid nodes remain";
        //skip another header
        // std::getline(file,line);
        // while(std::getline(file,line))
        // {
        //         adjGraph.push_back(parseGraphEntry(line));
        //
        // }
        file.close();
        return true;
}

std::vector<int> adjacencyMap::parseGraphEntry(std::string line)
{
        std::vector<int> entry;
        std::istringstream aiss(line);
        std::string tmp;
        std::getline(aiss,tmp,':');
        int cnt = std::stoi(tmp);
        while (std::getline(aiss,tmp,',')) {
                entry.push_back(std::stoi(tmp));
        }
        // //Debug
        // std::cout << cnt << ": ";
        // for (size_t i = 0; i < entry.size(); i++) {
        //         std::cout << entry[i] << " ";
        // }
        // std::cout << '\n';
        return entry;

}

pointGeom adjacencyMap::parseCodeLine(std::string line)
{
        std::istringstream iss(line);
        std::string tmp;
        float x,y,z;
        int label;
        std::getline(iss,tmp,',');
        x=std::stof(tmp.c_str());
        std::getline(iss,tmp,',');
        y=std::stof(tmp.c_str());
        std::getline(iss,tmp,',');
        z=std::stof(tmp.c_str());
        std::getline(iss,tmp,',');
        label = std::stoi(tmp);
        //DEbug
        //printf("%f,%f,%f,%d\n",x,y,z,label );
        pointGeom g = makePointGeom(x,y,z);
        return g;
}

bool adjacencyMap::pruneNode(pointGeom p, pointArray& tmpNodes)
{
        //Check if the node is sufficiently far awya from others
        //sphere collision between free nodes
        bool prune=false;
        if (tmpNodes.empty()) {
                //if grpah is empty add node
                std::cout << "Grpah is empty" << '\n';
                return prune;
        }
        Eigen::Vector3d pointEigen(p.x,p.y,p.z);
        Eigen::Vector3d node(tmpNodes[0].x,tmpNodes[0].y,tmpNodes[0].z);

        for (int i = 0; i < tmpNodes.size(); i++)
        {
                double distance = (pointEigen - node).norm();
                //std::cout << "Distance: " << distance << '\n';
                if (distance<minDist) {
                        prune = true;
                        break;
                }
                node << tmpNodes[i].x,tmpNodes[i].y,tmpNodes[i].z;
        }
        //If any node is closer to any other node by minDist then prune it
        return prune;
}

pointGeom adjacencyMap::makePointGeom(float x, float y, float z)
{
        pointGeom g;
        g.x=x;
        g.y=y;
        g.z=z;
        return g;
}

pointArray adjacencyMap::getFreeNodes()
{
        return this->freeNodes;
}

pointArray adjacencyMap::getOccNodes()
{
        return this->occupiedNodes;
}

adjacencyList adjacencyMap::getGraph()
{
        return this->adjGraph;
}

bool adjacencyMap::Astar(pointGeom goal,pointGeom start, pointArray & fullPath)
{

        int closestNodeStrt = getClosestNode(start);
        int closestNodeGoal = getClosestNode(goal);
        std::cout << "Closest node to start is: ";
        printPointGeom(freeNodes[closestNodeStrt]);
        std::cout << "Closest node to goal is: ";
        printPointGeom(freeNodes[closestNodeGoal]);
        const double maxCost = 1000;
        std::vector<double> accCost(adjGraph.size(),maxCost);
        std::vector<int> path(adjGraph.size(),-1);

        std::priority_queue<graphNode> pq;
        //double cost = euclideanDistance()
        pq.push(graphNode(closestNodeStrt,0));
        accCost[closestNodeStrt]=0;
        int nodesVisited =0;
        while (!pq.empty()) {
                int current = pq.top().vertex;
                pq.pop();
                nodesVisited++;
                //std::cout << "Current node" << current<<'\n';
                if (nodesVisited>freeNodes.size())
                {
                        std::cout << "Error could not find path, check connectivity" << '\n';
                        return false;
                }
                if (current == closestNodeGoal)
                {
                        break;
                }

                for (int nxt = 0; nxt < adjGraph[current].size(); nxt++) {
                        int nxtNode = adjGraph[current][nxt];
                        pointGeom nxtPoint = freeNodes[nxtNode];
                        double d =  euclideanDistance(freeNodes[current],
                                                      nxtPoint);
                        double newCost = d + accCost[current];
                        if (newCost<accCost[nxtNode]) {
                                accCost[nxtNode]=newCost;
                                double priority =
                                        newCost +
                                        euclideanDistance(nxtPoint,
                                                          freeNodes[closestNodeGoal]);
                                pq.push(graphNode(nxtNode,priority));
                                path[nxtNode]= current;
                        }
                }
        }

        fullPath.push_back(goal);
        int nd = closestNodeGoal;
        while (nd != closestNodeStrt) {
                printf("C[%d]\n",nd );
                printPointGeom(freeNodes[nd]);
                if (nd == -1) {
                        std::cout << "Path creation is  failure" << '\n';
                        return false;
                }
                fullPath.push_back(freeNodes[nd]);
                nd = path[nd];

        }
        fullPath.push_back(start);
        std::reverse(fullPath.begin(),fullPath.end());
        return true;
}

int adjacencyMap::getClosestNode(pointGeom p)
{
        //find closest node in graph to an arbitrary point
        Eigen::Vector3d pointEigen(p.x,p.y,p.z);
        Eigen::Vector3d node(freeNodes[0].x,freeNodes[0].y,freeNodes[0].z);
        double minDistance = (pointEigen - node).norm();
        int closestNode=0;
        for (int i = 0; i < freeNodes.size(); i++)
        {
                node << freeNodes[i].x,freeNodes[i].y,freeNodes[i].z;
                double distance = (pointEigen - node).norm();
                if (distance<minDistance) {
                        closestNode = i;
                        minDistance = distance;
                }
        }
        std::cout << "Closest node is" << closestNode <<'\n';
        return closestNode;
}



double adjacencyMap::euclideanDistance(pointGeom a, pointGeom b)
{
        Eigen::Vector3d eigenA(a.x,a.y,a.z);
        Eigen::Vector3d eigenB(b.x,b.y,b.z);
        return (eigenA-eigenB).norm();
}

void adjacencyMap::printPointGeom(pointGeom p)
{
        printf("[%f,%f,%f]\n",p.x,p.y,p.z );
}

void adjacencyMap::printAdjacencyList(adjacencyList l)
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


void adjacencyMap::makeGraph()
{
        //Allocate an initialize adj matrix
        std::cout << "Adj Map->Constructing adjacency graph" << '\n';
        if (freeNodes.empty()) {
                std::cout << "Adj Map-> Codebook is empty" << '\n';
                return;
        }
        for (size_t i = 0; i < adjGraph.size(); i++) {
                adjGraph[i].clear();
        }
        adjGraph.clear();
        adjGraph = adjacencyList(freeNodes.size());
        std::cout << "Using Free nodes "<<freeNodes.size() << '\n';
        //

        Knn(freeNodes,adjGraph);
        //Save to disk as file
        std::cout << "**********" <<  '\n';
        printAdjacencyList(adjGraph);
        std::cout << "**********" <<  '\n';
        //saveAdjGraph(graphFile,adjGraph);
        //makeVizMsgAndPublish(adjGraph);
        return;
}

void adjacencyMap::Knn(pointArray &centroids, adjacencyList & adjL)
{
        //calculate the k nearest neighborrs of the centroids
        //And return the adjacency graph
        int nCnt = centroids.size();
        std::vector<distanceLabel> distances(nCnt);
        std::cout << maxDist << '\n';
        for (int i = 0; i <nCnt; i++)
        {
                for (int j = 0; j < nCnt; j++)
                {
                        distances[j].dist=euclideanDistance(centroids[i],centroids[j]);
                        distances[j].label=j;
                }
                // std:: cout << "Checking code:" << '\n';
                // printf("C[%d]",i ); printPointGeom(centroids[i]);
                std::sort(distances.begin(),distances.end(),compareDistance);
                //Fisrt element is always the node compared
                //with itself, distance=0 by definition

                for(int k=1; k<kNeighboors+1; k++)
                {
                        float localDist =distances[k].dist;
                        if(localDist <= maxDist)
                        {
                                int label=distances[k].label; //wich centroid
                                if (!validateConnection(centroids[label],centroids[i],
                                                        connectionRadius))
                                {
                                        bool isPresent=(std::find(adjL[i].begin(),
                                                                  adjL[i].end(), label) ==  adjL[i].end() );
                                        if (isPresent) {

                                                adjL[i].push_back(label);
                                        }
                                        isPresent=(std::find(adjL[label].begin(),
                                                             adjL[label].end(), i) ==  adjL[label].end() );
                                        if ( isPresent)
                                        {

                                                adjL[label].push_back(i);
                                        }
                                }
                                // else
                                // {
                                //         std::cout << "Collision between" << '\n';
                                //         printf("C[%d]",i ); printPointGeom(centroids[i]);
                                //         printf("C[%d]",label ); printPointGeom(centroids[label  ]);
                                // }
                        }
                }

        }
        return;
}


bool adjacencyMap::validateConnection(pointGeom p1, pointGeom p2, float radius)
{
        //Chceck if a connection between two free centroids is intersected by an occupied
        //point
        Eigen::Vector3d p1Eigen(p1.x,p1.y,p1.z+radius);
        Eigen::Vector3d p2Eigen(p2.x,p2.y,p2.z+radius);
        //std::cout << p1Eigen << "<---->"<< p2Eigen<< '\n';
        bool collision = false;
        for (size_t i = 0; i < occupiedNodes.size(); i++) {
                Eigen::Vector3d q(occupiedNodes[i].x,occupiedNodes[i].y,occupiedNodes[i].z);
                collision = cylinderCollision(p1Eigen,p2Eigen,q,radius);
                if (collision) {
                        break;
                }
        }
        return collision;
}

bool adjacencyMap::validateNode(pointGeom p1)
{
        //Chceck if a point is too close to occupied node
        Eigen::Vector3d p1Eigen(p1.x,p1.y,p1.z);
        Eigen::Vector3d p2Eigen(p1.x,p1.y,p1.z+safetyHeight);
        //std::cout << p1Eigen << "<---->"<< p2Eigen<< '\n';
        bool collision = false;
        for (size_t i = 0; i < occupiedNodes.size(); i++) {
                Eigen::Vector3d q(occupiedNodes[i].x,occupiedNodes[i].y,occupiedNodes[i].z);
                collision = cylinderCollision(p1Eigen,p2Eigen,q,safetyRadius);
                if (collision) {
                        break;
                }
        }
        return collision;
}



bool adjacencyMap::cylinderCollision(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d q, float radius)
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

bool adjacencyMap::compareDistance(distanceLabel i, distanceLabel j)
{
        return i.dist<j.dist;
}
