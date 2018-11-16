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
        for (int i = 0; i < nFree; i++)
        {
                pointGeom code;
                std::getline(file,line);
                code = parseCodeLine(line);
                freeNodes.push_back(code);
        }
        //skip another header
        std::getline(file,line);
        while(std::getline(file,line))
        {
                adjGraph.push_back(parseGraphEntry(line));

        }

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

pointArray adjacencyMap::Astar(pointGeom goal,pointGeom start)
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

        while (!pq.empty()) {
                int current = pq.top().vertex;
                pq.pop();
                //std::cout << "Current node" << current<<'\n';
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

        pointArray fullPath;
        fullPath.push_back(goal);
        int nd = closestNodeGoal;
        while (nd != closestNodeStrt) {
                fullPath.push_back(freeNodes[nd]);
                nd = path[nd];
                std::cout << nd << '\n';
        }
        fullPath.push_back(start);
        std::reverse(fullPath.begin(),fullPath.end());
        return fullPath;
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
