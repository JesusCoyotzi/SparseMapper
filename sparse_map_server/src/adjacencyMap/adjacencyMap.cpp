#include "sparse_map_server/adjacencyMap.h"


adjacencyMap::graphNode::graphNode(int vertex, double cost)
{
        this->vertex = vertex;
        this->cost = cost;
}

bool adjacencyMap::graphNode::operator<(const graphNode &a) const
{
        return a.cost < this->cost;
}

adjacencyMap::adjacencyMap()
{
        //default empty constructor
        //default empty constructor
        //Nedded do not erase
        return;
}

bool adjacencyMap::distanceLabel::operator<(const distanceLabel& a) const
{
        return this->dist < a.dist;
}

adjacencyMap::adjacencyMap(std::string pcdFile, std::string edgesFiles,
                           float safeHeight,float safeRadius,
                           float mxDistTerm)
{

        //Setter for params
        this->safetyHeight = safeHeight;
        this->safetyRadius = safeRadius;
        this->maxDistTerm = mxDistTerm;
        std::cout << "Parameters set for adj map:" << '\n';
        std::cout << "Safety Heigth: " << safetyHeight  << '\n';
        std::cout << "Safety Radius: " << safetyRadius  << '\n';
        std::cout << "Max distance to terminal: " << maxDistTerm  << '\n';
        loadFullGraph(pcdFile,edgesFiles);
        std::cout << "**********" <<  '\n';
        printAdjacencyList(adjGraph);
        std::cout << "**********" <<  '\n';
        return;
}

adjacencyMap::adjacencyMap(std::string mapFileName)
{
        loadMap(mapFileName);
        return;
}

adjacencyMap::adjacencyMap(std::string mapFile,
                           float safeHeight,float safeRadius,
                           float cRadius, float mxDist, float minDist,
                           float mxDistTerm,
                           int kNeighboors)
{
        //Setter for params
        this->safetyHeight = safeHeight;
        this->safetyRadius = safeRadius;
        this->connectionRadius = cRadius;
        this->maxDist = mxDist;
        this->minDist = minDist;
        this->maxDistTerm = mxDistTerm;
        this->kNeighboors = kNeighboors;
        std::cout << "Parameters set for adj map:" << '\n';
        std::cout << "Safety Heigth: " << safetyHeight  << '\n';
        std::cout << "Safety Radius: " << safetyRadius  << '\n';
        std::cout << "Connection Radius: " << connectionRadius  << '\n';
        std::cout << "Max distance: " << maxDist  << '\n';
        std::cout << "Max distance to terminal: " << maxDistTerm  << '\n';
        std::cout << "Min distance: " << minDist  << '\n';
        std::cout << "K neighbors: " << this->kNeighboors  << '\n';

        size_t fnd = mapFile.find(".");
        std::string extension = mapFile.substr(fnd+1,mapFile.size());

        if (!extension.compare("pcd")) {
                std::cout << "Reading as PCD file" << '\n';
                loadFromPCD(mapFile);
        }
        else {
                std::cout << "Read as legacy txt graph" << '\n';
                loadMap(mapFile);
        }
        return;
}


void adjacencyMap::setParams(float safeHeight,float safeRadius,
                             float cRadius, float mxDist, float minDist,
                             int kNeighboors)
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

bool adjacencyMap::loadFullGraph(std::string pcdFile, std::string edgeFile)
{
        graphIO graphReader;
        if (!graphReader.readGraphFiles(pcdFile,edgeFile))
        {
                return false;
        }
        freeNodes = graphReader.getFreeCodes();
        occupiedNodes = graphReader.getOccCodes();
        std::cout << "Read: " << occupiedNodes.size() << "occupied nodes from file\n";
        std::cout << "Read: " << freeNodes.size() << "free nodes from file\n";
        adjGraph = graphReader.getEdges();
        return true;
}

bool adjacencyMap::loadFromPCD(std::string filename)
{
        //Loads nodes from PCD file
        graphIO graphReader;
        if(!graphReader.loadPCD(filename))
        {
                return false;
        }

        std::list<pointGeom> tmpFree, tmpOcc;
        pointArray reducedNodes;
        graphReader.getNodes(tmpOcc,tmpFree);
        std::cout << "Read: " << tmpOcc.size() << "occupied nodes from file\n";
        std::cout << "Read: " << tmpFree.size() << "free nodes from file\n";
        //Prune all nodes higher than robot height
        for (pointGeom  &p : tmpOcc)
        {
                if (p.z<=safetyHeight) {
                        occupiedNodes.push_back(p);
                }
        }
        std::cout << "Only: " << occupiedNodes.size() <<"significant occupied nodes remain\n";
        //Reduce redundant free nodes
        reduceNodes(tmpFree,reducedNodes);
        std::cout << "Only: " << reducedNodes.size() <<"non-redundant free nodes remain\n";

        for (size_t i = 0; i < reducedNodes.size(); i++) {
                bool isValidNode = !validateNode(reducedNodes[i]);
                if (isValidNode)
                {
                        freeNodes.push_back(reducedNodes[i]);
                }
        }

        std::cout << "Only " << freeNodes.size() <<" valid nodes remain\n";
        return true;
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
        std::cout << "Listed: " <<nOcc<< " occupied centroids" <<'\n';
        //****Occupied nodes
        pointGeom code;
        std::getline(file,line);
        while (parseCodeLine(line,code))
        {
                if (code.z <= safetyHeight)
                {
                        /* If occupied node is higher than the robot height, why
                           do we even bother?*/
                        occupiedNodes.push_back(code);
                }
                std::getline(file,line);
        }
        std::cout << "Read: " << occupiedNodes.size()<< " occupied nodes while reading\n";

        //Number of free nodes
        //std::getline(file,line);
        iss.clear();
        iss.str(std::string());
        iss.str(line);
        std::getline(iss, tmp, ':');
        iss>>nFree;
        std::cout << "Listed: " <<nFree<< " free centroids" <<'\n';
        //****Free nodes
        std::list<pointGeom> freeList;
        pointArray tempFree;
        std::getline(file,line);
        while (parseCodeLine(line,code)) {
                freeList.push_back(code);
                // if (!pruneNode(code,tempFree)) {
                // }
                std::getline(file,line);
        }
        std::cout << "Read: " << freeList.size()<< " actual free nodes while reading\n";
        reduceNodes(freeList,tempFree);
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

int adjacencyMap::reduceNodes(std::list<pointGeom> &nodes, pointArray &oNodes)
{
        /* Reads a full list of points, reduces them using a voxel like filter
           create 3d grid based on min_dist, and loop all points over it
           for every box check how many points are inside and get the centroid
           oNodes is all the centroids calculated*/
        //Get largest and smalles element in nodes

        std::list<pointGeom>::iterator i;
        while (!nodes.empty()) {
                pointGeom q = nodes.front();
                nodes.pop_front();
                //std::cout << nodes.size() << '\n';
                Eigen::Vector3d query(q.x,q.y,q.z);
                Eigen::Vector3d accumulator = query;
                int n = 1;
                i = nodes.begin();
                while (i != nodes.end())
                {
                        pointGeom workingN = *i;
                        Eigen::Vector3d workingNode(workingN.x,workingN.y,workingN.z);
                        float dist = (workingNode-query).norm();
                        if (dist<minDist)
                        {
                                accumulator += workingNode;
                                i = nodes.erase(i);
                                n++;
                        }
                        else
                        {
                                ++i;
                        }
                }
                if (n>0)
                {
                        accumulator /= n;
                        pointGeom centroid = makePointGeom(accumulator(0),accumulator(1),accumulator(2));
                        oNodes.push_back(centroid);
                        //std::cout << centroid << '\n';
                }
        }
        return oNodes.size();
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

bool adjacencyMap::parseCodeLine(std::string line,  pointGeom &g)
{
        std::istringstream iss(line);
        std::string tmp;
        float x,y,z;
        int label;
        bool sux = true;
        try {
                std::getline(iss,tmp,',');
                x=std::stof(tmp.c_str());
                std::getline(iss,tmp,',');
                y=std::stof(tmp.c_str());
                std::getline(iss,tmp,',');
                z=std::stof(tmp.c_str());
                std::getline(iss,tmp,',');
                label = std::stoi(tmp);
        }
        catch (std::invalid_argument e)
        {
                // std::cout << "An exception occurred: " << e.what() << '\n';
                std::cout << line << '\n';
                x=0;  y=0;  z=0;
                sux = false;
        }
        //DEbug
        //printf("%f,%f,%f,%d\n",x,y,z,label );
        g = makePointGeom(x,y,z);
        return sux;
}

bool adjacencyMap::pruneNode(pointGeom p, pointArray& tmpNodes)
{
        //Check if the node is sufficiently far awya from others
        //sphere collision between free nodes
        bool prune=false;
        if (tmpNodes.empty()) {
                //if grpah is empty add node
                //std::cout << "Grpah is empty" << '\n';
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

bool adjacencyMap::removeOccupiedPoint(pointGeom p)
{
        //Find closest node to p
        int closestIdx = getClosestOccNode(p);
        bool suxes = false;
        float dist = euclideanDistance(occupiedNodes[closestIdx],p);
        std::cout << dist << '\n';
        if ( dist < maxDist)
        {
                //If farther than maxDist i just clicked a random point do nothing
                //Else remove point
                std::cout << "Removing: " << occupiedNodes[closestIdx]<< '\n';
                occupiedNodes.erase(occupiedNodes.begin() + closestIdx);
                suxes = true;
        }
        return suxes;
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

adjacencyList adjacencyMap::getEdges()
{
        return this->adjGraph;
}

bool adjacencyMap::Astar(int goal,int start, pointArray & fullPath)
{
        /*get shortes path using A* algorithm
           goal  index of goal node
           start index of start node
           fullPath reconstructed path*/
        // int closestNodeStrt = getClosestNode(start);
        // int closestNodeGoal = getClosestNode(goal);
        // std::cout << "Closest node to start is: ";
        // printPointGeom(freeNodes[closestNodeStrt]);
        // std::cout << "Closest node to goal is: ";
        // printPointGeom(freeNodes[closestNodeGoal]);
        pointGeom goalPoint =freeNodes[goal];
        pointGeom startPoint =freeNodes[start];

        const double maxCost = 1000.0;
        std::vector<double> accCost(adjGraph.size(),maxCost);
        std::vector<int> path(adjGraph.size(),-1);
        std::priority_queue<graphNode> pq;
        //double cost = euclideanDistance()
        pq.push(graphNode(start,0));
        accCost[start]=0;
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
                if (current == goal)
                {
                        break;
                }

                for (int nxt = 0; nxt < adjGraph[current].size(); nxt++)
                {
                        int nxtNode = adjGraph[current][nxt];
                        pointGeom nxtPoint = freeNodes[nxtNode];
                        double d =  euclideanDistance(freeNodes[current],
                                                      nxtPoint);
                        double newCost = d + accCost[current];
                        if (newCost<accCost[nxtNode])
                        {
                                accCost[nxtNode]=newCost;
                                double priority =
                                        newCost +
                                        euclideanDistance(nxtPoint,goalPoint);
                                pq.push(graphNode(nxtNode,priority));
                                path[nxtNode]= current;
                        }
                }
        }

        int nd = goal;
        while (nd != start) {
                // printf("C[%d]\n",nd );
                // printPointGeom(freeNodes[nd]);
                if (nd == -1) {
                        std::cout << "Error: Path not found" << '\n';
                        return false;
                }
                fullPath.push_back(freeNodes[nd]);
                nd = path[nd];

        }
        fullPath.push_back(startPoint);
        std::reverse(fullPath.begin(),fullPath.end());
        return true;
}

int adjacencyMap::getClosestNode(pointGeom p)
{
        //find closest node in graph to an arbitrary point
        Eigen::Vector3d pointEigen(p.x,p.y,p.z);
        Eigen::Vector3d node(freeNodes[0].x,freeNodes[0].y,freeNodes[0].z);
        double smallestDistance = (pointEigen - node).squaredNorm();
        int closestNode=0;
        for (int i = 0; i < freeNodes.size(); i++)
        {
                node << freeNodes[i].x,freeNodes[i].y,freeNodes[i].z;
                double distance = (pointEigen - node).squaredNorm();
                if (distance < smallestDistance)
                {
                        //If closest node is too close ignore it
                        //Take closest node outside robot radius
                        //Sadly does not always works as intended :(
                        closestNode = i;
                        smallestDistance = distance;
                        // if(distance>minDist)
                        // {
                        // }
                }
        }
        //std::cout << "Closest node is" << closestNode <<'\n';
        return closestNode;
}

int adjacencyMap::getClosestOccNode(pointGeom p)
{
        Eigen::Vector3d pointEigen(p.x,p.y,p.z);
        Eigen::Vector3d node(occupiedNodes[0].x,occupiedNodes[0].y,occupiedNodes[0].z);
        double minDistance = (pointEigen - node).norm();
        int closestNode=0;
        for (int i = 0; i < occupiedNodes.size(); i++)
        {
                node << occupiedNodes[i].x,occupiedNodes[i].y,occupiedNodes[i].z;
                double distance = (pointEigen - node).norm();
                if (distance<minDistance) {
                        closestNode = i;
                        minDistance = distance;
                }
        }
        //std::cout << "Closest node is" << closestNode <<'\n';
        return closestNode;
}

float adjacencyMap::getClosestNodeDistance(pointGeom p)
{
        //get the distance to closes node in graph
        int nodeIdx = getClosestNode(p);
        return euclideanDistance(freeNodes[nodeIdx],p);
}


double adjacencyMap::euclideanDistance(pointGeom a, pointGeom b)
{
        Eigen::Vector3d EigenA(a.x,a.y,a.z);
        Eigen::Vector3d EigenB(b.x,b.y,b.z);
        return (EigenA-EigenB).norm();
}

double adjacencyMap::L1Distance(pointGeom a, pointGeom b)
{
        Eigen::Vector3d EigenA(a.x,a.y,a.z);
        Eigen::Vector3d EigenB(b.x,b.y,b.z);
        return (EigenA-EigenB).squaredNorm();
}

double adjacencyMap::chebyshevDistance(pointGeom a, pointGeom b)
{
        double dx = abs(a.x-b.x);
        double dy = abs(a.y-b.y);
        return (dx>dy) ? dx : dy;
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
        for (int i = 0; i <nCnt; i++)
        {
                for (int j = 0; j < nCnt; j++)
                {
                        distances[j].dist=euclideanDistance(centroids[i],centroids[j]);
                        distances[j].label=j;
                }
                // std:: cout << "Checking code:" << '\n';
                // printf("C[%d]",i ); printPointGeom(centroids[i]);
                std::sort(distances.begin(),distances.end());
                //Fisrt element is always the node compared
                //with itself, distance=0 by definition

                //Since connection is undirected, it can happen some nodes en with more
                //than kNeighboors
                for(int k=1; k<kNeighboors+1; k++)
                {
                        float localDist =distances[k].dist;
                        if(localDist <= maxDist)
                        {
                                int label=distances[k].label; //wich centroid
                                if (!validateConnection(centroids[label],centroids[i],
                                                        connectionRadius))
                                {
                                        //Check if node has been previously added else ignore
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
                        else
                        {
                                break;
                        }
                }

        }
        return;
}


bool adjacencyMap::validateConnection(pointGeom p1, pointGeom p2, float radius)
{
        //Chceck if a connection between two free centroids is intersected by an occupied
        //point
        Eigen::Vector3d p1Eigen(p1.x,p1.y,p1.z+radius); //why radiuus? cylinder bvetwwen connections!
        Eigen::Vector3d p2Eigen(p2.x,p2.y,p2.z+radius);
        //std::cout << p1Eigen << "<---->"<< p2Eigen<< '\n';
        // std::vector<distanceLabel> distances(nCnt);
        // for (int j = 0; j < occupiedNodes.size(); j++)
        // {
        //         distances[j].dist=euclideanDistance(centroids[i],centroids[j]);
        //         distances[j].label=j;
        // }
        bool collision = false;
        for (size_t i = 0; i < occupiedNodes.size(); i++) {
                Eigen::Vector3d q(occupiedNodes[i].x,occupiedNodes[i].y,occupiedNodes[i].z);
                //Check if it is worth checking collision

                collision = cylinderCollision(p1Eigen,p2Eigen,q,radius);
                if (collision) {
                        break;
                }
        }
        return collision;
}

bool adjacencyMap::validateNode(pointGeom p1)
{
        //Check if a point is too close to occupied node
        //Returns true if collision exists
        Eigen::Vector3d p1Eigen(p1.x,p1.y,p1.z);
        Eigen::Vector3d p2Eigen(p1.x,p1.y,p1.z+safetyHeight);
        //std::cout << p1Eigen << "<---->"<< p2Eigen<< '\n';
        float minimalDist = (safetyHeight*safetyHeight+safetyRadius*safetyRadius);
        bool collision = false;
        for (size_t i = 0; i < occupiedNodes.size(); i++) {
                Eigen::Vector3d q(occupiedNodes[i].x,occupiedNodes[i].y,occupiedNodes[i].z);
                //Check if it is worth doing cilinder collision
                //If node is closer than safetyH + safetyRadius check else ignore
                float dist = (p1Eigen-q).squaredNorm();
                if (dist < minimalDist) {
                        collision = cylinderCollision(p1Eigen,p2Eigen,q,safetyRadius);
                        if (collision) {
                                break;
                        }
                }
        }
        return collision;
}

bool adjacencyMap::validateNode(pointGeom p, pointArray &codes)
{
        //Like above but this,checks with a subset of codes instead of all of them
        Eigen::Vector3d p1Eigen(p.x,p.y,p.z);
        Eigen::Vector3d p2Eigen(p.x,p.y,p.z+safetyHeight);
        //std::cout << p1Eigen << "<---->"<< p2Eigen<< '\n';
        float minimalDist = (safetyHeight*safetyHeight+safetyRadius*safetyRadius);
        bool collision = false;
        for (size_t i = 0; i < codes.size(); i++) {
                Eigen::Vector3d q(codes[i].x,codes[i].y,codes[i].z);
                //Check if it is worth doing cilinder collision
                //If node is closer than safetyH + safetyRadius check else ignore
                float dist = (p1Eigen-q).squaredNorm();
                if (dist < minimalDist) {
                        collision = cylinderCollision(p1Eigen,p2Eigen,q,safetyRadius);
                        if (collision) {
                                break;
                        }
                }
        }
        //std::cout << "The object collides " << collision<< '\n';
        return collision;
}

bool adjacencyMap::validateSingleTerminal(pointGeom p, int &nodeIdx, pointArray &codesToCheck)
{
        nodeIdx = getClosestNode(p);
        float dst = L1Distance(freeNodes[nodeIdx],p);
        float sqr = maxDistTerm*maxDistTerm;
        if (dst > sqr) {
                std::cout << "Error: Start node is " << dst << " [m] away from closest node\n";
                std::cout << "Max allowed distance is:" << maxDistTerm<< "[m]\n";
                return false;
        }

        bool isNotValid = validateNode(p,codesToCheck);
        return !isNotValid;
}


bool adjacencyMap::validateTerminals(pointGeom strt, pointGeom goal,
                                     int &closetsNodeStrtIdx, int &closestNodeGoalIdx)
{
        closetsNodeStrtIdx = getClosestNode(strt);
        float strtDst =euclideanDistance(freeNodes[closetsNodeStrtIdx],strt);
        if (strtDst > maxDistTerm) {
                std::cout << "Error: Start node is " << strtDst << " [m] away from closest node\n";
                std::cout << "Max allowed distance is:" << maxDistTerm<< "[m]\n";
                return false;
        }

        closestNodeGoalIdx = getClosestNode(goal);
        float goalDst =euclideanDistance(freeNodes[closestNodeGoalIdx],goal);
        if (goalDst > maxDistTerm) {
                std::cout << "Error: Goal node is " << goalDst << " [m] away from closest node\n";
                std::cout << "Max allowed distance is:" << maxDistTerm<< "[m]\n";
                return false;
        }

        bool isNotValid = validateNode(strt) || validateNode (goal);
        return !isNotValid;
}

bool adjacencyMap::validateTerminalsQuick(pointGeom strt, pointGeom goal,
                                          int &closetsNodeStrtIdx, int &closestNodeGoalIdx)
{
        closetsNodeStrtIdx = getClosestNode(strt);
        float strtDst =euclideanDistance(freeNodes[closetsNodeStrtIdx],strt);
        if (strtDst > maxDistTerm) {
                std::cout << "Error: Start node is " << strtDst << " [m] away from closest node\n";
                std::cout << "Max allowed distance is:" << maxDist<< "[m]\n";
                return false;
        }

        closestNodeGoalIdx = getClosestNode(goal);
        float goalDst =euclideanDistance(freeNodes[closestNodeGoalIdx],goal);
        if (goalDst > maxDistTerm) {
                std::cout << "Error: Goal node is " << goalDst << " [m] away from closest node\n";
                std::cout << "Max allowed distance is:" << maxDist<< "[m]\n";
                return false;
        }

        return true;
}


bool adjacencyMap::cylinderCollision(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d q, float radius)
{
        Eigen::Vector3d p2_p1 = p2-p1;
        // std::cout << "Conecttions from " << '\n';
        // std::cout << "p1: " <<  p1<<'\n';
        // std::cout << "p2: " <<  p2<<'\n';
        // std::cout << "q: " <<  q<<'\n';
        if (p2_p1.norm()<0.001) {
                std::cout << "Same point no cylinder " <<  p2_p1.norm() <<'\n';
                return true;
        }
        //TODO change order of checking
        //Radius first
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
