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

void adjacencyMap::sayHi()
{
        std::cout << "Helog there i am the functions" << '\n';
        return;
}
