#include "sparse_map_server/adjacencyMap.h"

graphIO::graphIO()
{
        return;
}

graphIO::graphIO(std::string filename)
{
        loadNodes(filename);
        return;
}

bool graphIO::loadNodes(std::string filename)
{
        //  std::cout << "Loading map from" << filename <<'\n';
        std::ifstream file(filename.c_str());
        if(!file.is_open())
        {
                std::cout << "Error reading file:" << filename <<'\n';
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
        //Occupied nodes
        pointGeom code;
        std::getline(file,line);
        while (parseCodeLine(line,code)) {
                occCodes.push_back(code);
                std::getline(file,line);
        }
        std::cout << "Read: " << occCodes.size()<< " occupied nodes while reading\n";

        //Number of free nodes
        //std::getline(file,line);
        iss.clear();
        iss.str(std::string());
        iss.str(line);
        std::getline(iss, tmp, ':');
        iss>>nFree;
        std::cout << "Listed: " <<nFree<< " free centroids" <<'\n';
        //Free nodes
        std::getline(file,line);
        while (parseCodeLine(line,code)) {
                freeCodes.push_back(code);
                std::getline(file,line);
        }
        //std::cout << "Read: " << tempFree.size()<< " free nodes while reading\n";
        std::cout << "Found " << freeCodes.size() <<" valid nodes remains\n";
        file.close();
        return true;
}

bool graphIO::loadPCD(std::string filename)
{
        std::ifstream file(filename.c_str());
        if(!file.is_open())
        {
                std::cout << "Error reading file:" << filename <<'\n';
                return false;
        }
        std::string line;
        std::istringstream iss(line);

        for (size_t i = 0; i < 11; i++) {
                std::getline(file,line);
        }

        iss.clear();
        iss.str(std::string());
        iss.str(line);
        pointGeom code;
        std::getline(file,line);
        int type =0;
        while (parsePCDLine(line,code,type)) {
                if (type == 0) {
                        occCodes.push_back(code);
                }
                else if(type == 1)
                {
                        freeCodes.push_back(code);
                }

                std::getline(file,line);
        }
        //Once nodes are loaded resize adjacency list
        graph.resize(freeCodes.size());
        return true;
}

bool graphIO::saveAsPCD(std::string filename)
{
        //Writes as ascii pcd file
        std::ofstream file(filename.c_str());
        if(!file.is_open())
        {
                std::cout << "Error writing to file:" << filename <<'\n';
                return false;
        }
        //Write pcd header
        file << "# Quantization nodes!\n";
        file << "VERSION .7\n";
        file << "FIELDS x y z label\n";
        file << "SIZE 4 4 4 4\n";
        file << "TYPE F F F I\n";
        file << "COUNT 1 1 1 1\n";
        file << "WIDTH " << freeCodes.size() + occCodes.size() << std::endl;
        file << "HEIGHT 1\n";
        file << "VIEWPOINT 0 0 0 1 0 0 0\n";
        file << "POINTS " << freeCodes.size() + occCodes.size()  << std::endl;
        file << "DATA ascii\n";
        //DATA
        for (pointGeom const &p : occCodes) {
                file<<p.x << " " << p.y << " " << p.z << " 0" << std::endl;
        }

        for (pointGeom const &p : freeCodes) {
                file<<p.x << " " << p.y << " " << p.z << " 1" << std::endl;
        }

        file.close();
        return true;
}

void graphIO::getNodes(std::list<pointGeom>& occNodes, std::list<pointGeom>& freeNodes)
{
        //Gets nodes inside structure.
        occNodes = this->occCodes;
        freeNodes = this->freeCodes;
}

adjacencyList graphIO::getEdges()
{
        return graph;
}

void graphIO::loadMap(adjacencyMap &graph)
{
        //Loads adjacency graph into a graphIO object to save to disk
        pointArray occ = graph.getOccNodes();
        std::copy( occ.begin(), occ.end(), std::back_inserter( occCodes  ) );
        pointArray fre = graph.getFreeNodes();
        std::copy( fre.begin(), fre.end(), std::back_inserter( freeCodes ) );
        this->graph = graph.getEdges();
        return;
}

bool graphIO::loadEdges(std::string graphFile)
{
        //Load edges from a plain text grph file
        std::ifstream file(graphFile.c_str());
        if(!file.is_open())
        {
                std::cout << "Error reading from edges file:" << graphFile <<'\n';
                return false;
        }

        //Skip header and location
        std::string line;
        std::istringstream iss(line);
        for (size_t i = 0; i < 2; i++)
        {
                std::getline(file,line);
        }

        iss.clear();
        iss.str(std::string());
        iss.str(line);

        int src = 0;
        std::vector<int> dst;
        std::cout << "Parsing edges" << '\n';
        std::getline(file,line);
        while (parseEdgeLine(line,src,dst)) {
                graph[src]=dst;
                std::getline(file,line);

        }
        file.close();
        return true;
}

bool graphIO::readGraphFiles(std::string pcdFile, std::string graphFile)
{
        //Loads an adjacency map from disk
        /*
           Takes a PCD and a .grph file from disk to ensemble the graphIO object
           pcdFile PCD file containing graph nodes.
           graphFile grph file containing the edges.
         */
        if (!loadPCD(pcdFile))
        {
                return false;
        }

        if(!loadEdges(graphFile))
        {
                return false;
        }
        if (freeCodes.size() != graph.size()) {
                std::cout << "WARNING: mismatch between nodes and edges size:" << '\n';
                std::cout << "# of Nodes: " << freeCodes.size()  <<'\n';
                std::cout << "# of Edges: " << graph.size()  <<'\n';
                return false;
        }

        return true;

}

bool graphIO::saveGraph(std::string filename)
{
        //Saves nodes and graph strcuture to disk

        size_t fnd = filename.find(".");
        std::string pcdName = filename.substr(0,fnd)+".pcd";
        bool sux = saveAsPCD(pcdName);
        if (!sux)
        {
                std::cout << "Error could not save nodes to file" << pcdName<< '\n';
                return false;
        }

        //TODO: create extra saveEdges function with this:
        std::ofstream file(filename.c_str());
        if(!file.is_open())
        {
                std::cout << "Error writing to graph file:" << filename <<'\n';
                return false;
        }

        file << "Graph file v 2\n";
        file << "Nodes file: " << pcdName << "\n";

        for (size_t i = 0; i < graph.size(); i++)
        {
                file << i << " ";
                for (int const & dstNode : graph[i])
                {
                        file<< dstNode << " ";
                }
                file << "\n";
        }


        file.close();
        return true;


}

pointArray graphIO::getFreeCodes()
{
        pointArray freeVector;
        std::copy( freeCodes.begin(), freeCodes.end(),
                   std::back_inserter( freeVector ) );
        return freeVector;

}


pointArray graphIO::getOccCodes()
{
        pointArray occVector;
        std::copy( occCodes.begin(), occCodes.end(),
                   std::back_inserter( occVector ) );
        return occVector;
}

bool graphIO::parseCodeLine(std::string line,  pointGeom &g)
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

        g.x=x; g.y=y; g.z=z;
        return sux;
}

bool graphIO::parsePCDLine(std::string line,  pointGeom &g, int & label)
{
        std::istringstream iss(line);
        std::string tmp;
        float x,y,z;
        bool sux = true;
        try {
                std::getline(iss,tmp,' ');
                x=std::stof(tmp.c_str());
                std::getline(iss,tmp,' ');
                y=std::stof(tmp.c_str());
                std::getline(iss,tmp,' ');
                z=std::stof(tmp.c_str());
                std::getline(iss,tmp,' ');
                label = std::stoi(tmp);
        }
        catch (std::invalid_argument e)
        {
                // std::cout << "An exception occurred: " << e.what() << '\n';
                std::cout << line << '\n';
                x=0;  y=0;  z=0;
                sux = false;
        }

        g.x=x; g.y=y; g.z=z;
        return sux;
}

bool graphIO::parseEdgeLine(std::string line,int &src, std::vector<int> &dst)
{
        std::istringstream iss(line);
        dst.clear();
        std::string tmp;
        bool sux = true;
        try {                
                std::getline(iss,tmp,' ');
                src=std::stoi(tmp.c_str());
                while (std::getline(iss,tmp,' '))
                {
                        int destiny = std::stoi(tmp.c_str());
                        dst.push_back(destiny);

                }
        }
        catch (std::invalid_argument e)
        {
                std::cout << "An exception occurred: " << e.what() << '\n';
                sux = false;
        }

        return sux;
}


pointGeom graphIO::removeOccCode(pointGeom p)
{
        Eigen::Vector3d pEigen(p.x,p.y,p.z);
        pointGeom closestPoint = occCodes.front(); occCodes.pop_front();
        Eigen::Vector3d cp(closestPoint.x,closestPoint.y,closestPoint.z);
        float minDist = (pEigen-cp).norm();
        std::list<pointGeom>::iterator closestIt;
        for (std::list<pointGeom>::iterator it = occCodes.begin();
             it != occCodes.end(); ++it)
        {
                Eigen::Vector3d workingCode(it->x,it->y,it->z);
                float dist = (workingCode-pEigen).norm();
                if (dist < minDist) {
                        minDist = dist;
                        closestPoint = *it;
                        closestIt = it;
                }
        }

        occCodes.erase(closestIt);
        return closestPoint;
}

pointGeom graphIO::removeFreeCode(pointGeom p)
{
        Eigen::Vector3d pEigen(p.x,p.y,p.z);
        pointGeom closestPoint = freeCodes.front(); freeCodes.pop_front();
        Eigen::Vector3d cp(closestPoint.x,closestPoint.y,closestPoint.z);
        float minDist = (pEigen-cp).norm();
        std::list<pointGeom>::iterator closestIt;
        for (std::list<pointGeom>::iterator it = freeCodes.begin();
             it != freeCodes.end(); ++it)
        {
                Eigen::Vector3d workingCode(it->x,it->y,it->z);
                float dist = (workingCode-pEigen).norm();
                if (dist < minDist) {
                        minDist = dist;
                        closestPoint = *it;
                        closestIt = it;
                }
        }

        freeCodes.erase(closestIt);
        return closestPoint;
}

void graphIO::addFreeCode(pointGeom p)
{
        freeCodes.push_back(p);
        return;
}

void graphIO::addOccCode(pointGeom p)
{
        occCodes.push_back(p);
        return;
}

bool graphIO::saveAsTxt(std::string filename)
{
        //Saves the adjacency matrix to a file to disk.
        std::cout << "Writing to "<<filename<<"\n";
        std::ofstream graphOut;
        int freeNodes =freeCodes.size(); int occNodes =  occCodes.size();
        int totalNodes = freeNodes+occNodes;
        graphOut.open(filename);
        if (!graphOut.is_open())
        {
                std::cout << "Error writing to:"<< filename << '\n';
                return false;
        }
        graphOut<<"Clusters: " << totalNodes << "\n";
        graphOut<<"Codebook: x,y,z,label\n";
        graphOut << "Occupied Nodes: " << occNodes <<"\n";

        freeCodes.sort(lessL1());
        occCodes.sort(lessL1());
        int i =0;
        for (std::list<pointGeom>::iterator it = occCodes.begin();
             it != occCodes.end(); ++it)
        {
                graphOut<<it->x<<",";
                graphOut<<it->y<<",";
                graphOut<<it->z<<",";
                graphOut<<i<<"\n"; i++;
        }

        i=0;
        graphOut << "Free Nodes:" << freeNodes<<"\n";
        for (std::list<pointGeom>::iterator it = freeCodes.begin();
             it != freeCodes.end(); ++it)
        {
                graphOut<<it->x<<",";
                graphOut<<it->y<<",";
                graphOut<<it->z<<",";
                graphOut<<i<<"\n"; i++;
        }
        graphOut.close();
        return true;
}

int graphIO::simpleOccZPassThrough(double max, double min)
{
        // A simple implementation of a passthorugh filter like pcl::passthorugh
        std::list<pointGeom>::iterator i;
        pointGeom q = occCodes.front();
        i = occCodes.begin();
        int n=0;
        while (i != occCodes.end())
        {
                double height = i->z;
                if ((height > max) || (height < min) )
                {
                        i = occCodes.erase(i);
                        n++;
                }
                else
                {
                        ++i;
                }
        }
        return n; //how many elements i pruned?

}

int graphIO::simpleFreeZPassThrough(double max, double min)
{
        // A simple implementation of a passthorugh filter like pcl::passthorugh
        std::list<pointGeom>::iterator i;
        pointGeom q = freeCodes.front();
        i = freeCodes.begin();
        int n=0;
        while (i != freeCodes.end())
        {
                double height = i->z;
                if ((height > max) || (height < min) )
                {
                        i = occCodes.erase(i);
                        n++;
                }
                else
                {
                        ++i;
                }
        }
        return n; //how many elements i pruned?

}

int graphIO::simpleZPassThrough(double max, double min)
{
        // A simple implementation of a passthorugh filter like pcl::passthorugh
        std::list<pointGeom>::iterator i;
        i = occCodes.begin();
        int n=0;
        while (i != occCodes.end())
        {
                double height = i->z;
                if ((height > max) || (height < min) )
                {
                        i = occCodes.erase(i);
                        n++;
                }
                else
                {
                        ++i;
                }
        }

        i = freeCodes.begin();
        while (i != freeCodes.end())
        {
                double height = i->z;
                if ((height > max) || (height < min) )
                {
                        i = freeCodes.erase(i);
                        n++;
                }
                else
                {
                        ++i;
                }
        }


        return n; //how many elements i pruned?

}
