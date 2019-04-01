#include "sparse_mapper/sparse_mapper.h"

sparseMapper::sparseMapper(ros::NodeHandle &nh)
{
        nh_=nh;
        ros::NodeHandle nh_priv("~");
        nh_priv.param<float>("free_thr",freeThr,0.1);
        // nh_priv.param<std::string>("graph_file",graphFile,"adjGraph.txt");

        codebookSub = nh_.subscribe("codebook",10, &sparseMapper::codebookCallback, this);
        graphMakeSub = nh_.advertiseService("make_map", &sparseMapper::saveMapSrv,this);
        pcdMakeSub = nh_.advertiseService("make_pcd", &sparseMapper::saveMapPCD,this);
        graphClearSub = nh_.subscribe("clear_graph",1, &sparseMapper::clearGraph,this);
        codebookMarkerPub=
                nh_.advertise<visualization_msgs::Marker>("centroids_marker",2);
        std::cout << "Starting sparse_mapper node by CoyoSoft" << '\n';

}

void sparseMapper::clearGraph(const std_msgs::Empty &msg)
{
        std::cout << "!!!Clearing all codes!!!!" << '\n';
        occCodebook.clear();
        freeCodebook.clear();
        //makeCentroidsMarkerAndPublish(freeCodebook,true);
}

std_msgs::ColorRGBA sparseMapper::makeColor(float r,float g, float b, float a)
{
        std_msgs::ColorRGBA color;
        color.r=r;
        color.g=g;
        color.b=b;
        color.a=a;
        return color;
}

void sparseMapper::makeCentroidsMarkerAndPublish(pointArray &codebook, std_msgs::ColorRGBA color,int id)
{
        const float radius = 0.05;
        visualization_msgs::Marker centroidsMarker;
        //centroidsMarker.points.resize(codebook.size());
        centroidsMarker.ns = "centroids_static";
        centroidsMarker.action = visualization_msgs::Marker::ADD;
        centroidsMarker.header.frame_id = codebookFrame;
        centroidsMarker.header.stamp = ros::Time();
        centroidsMarker.type = visualization_msgs::Marker::CUBE_LIST;
        centroidsMarker.pose.orientation.w = 1.0;
        centroidsMarker.scale.x = radius;
        centroidsMarker.scale.y = radius;
        centroidsMarker.scale.z = radius;
        centroidsMarker.id =id;
        centroidsMarker.color = color;
        centroidsMarker.points = codebook;
        codebookMarkerPub.publish(centroidsMarker);
        return;
}


void sparseMapper::codebookCallback(const sparse_map_msgs::codebook &msg)
{
        codebookFrame = msg.header.frame_id;
        stamp = msg.header.stamp;
        //TODO use remove erase idiom

        pointArray centroids = msg.centroids;
        for (int i = 0; i < centroids.size(); i++)
        {
                pointGeom cnt(centroids[i]);
                if (cnt.z>=freeThr)
                {
                        occCodebook.push_back(cnt);
                        //centroids.erase(centroids.begin()+i);
                }
        }
        for (int i = 0; i < centroids.size(); i++)
        {
                pointGeom cnt(centroids[i]);
                if (cnt.z<freeThr) {

                        freeCodebook.push_back(cnt);

                }
        }

        edges = freeCodebook.size()+occCodebook.size();
        std::cout << "Got a codebook of:" << centroids.size() << " points"<< '\n';
        std::cout << "In frame " << codebookFrame << " on Stamp " << stamp << "\n";
        std::cout << "Global map is " << freeCodebook.size() << "points long\n";
        std::cout << "Occupied space is " << occCodebook.size()<< "points long\n";

        std_msgs::ColorRGBA freeColor = makeColor(0,1,0.2,1);
        std_msgs::ColorRGBA occColor = makeColor(1,0.2,0,1);
        makeCentroidsMarkerAndPublish(freeCodebook, freeColor,0);
        makeCentroidsMarkerAndPublish(occCodebook, occColor,1);
        return;
}


bool sparseMapper::saveSparseMap(std::string filename)
{
        //Saves the adjacency matrix to a file to disk.
        std::cout << "Writing to "<<filename<<"\n";
        std::ofstream graphOut;
        int freeNodes =freeCodebook.size(); int occNodes =  occCodebook.size();
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

        std::sort(freeCodebook.begin(),freeCodebook.end(),lessL1());
        std::sort(occCodebook.begin(),occCodebook.end(),lessL1());
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
        graphOut.close();
        return true;
}

bool sparseMapper::saveMapSrv(sparse_map_msgs::SaveMap::Request &req,
                              sparse_map_msgs::SaveMap::Response &res)
{
        res.success = saveSparseMap(req.filename);
        if (res.success)
        {
                std::cout << "File saving successfull " << req.filename << std::endl;
        }
        else
        {
                std::cout << "Error saving " << req.filename << std::endl;

        }
        return true;
}

bool sparseMapper::saveMapPCD(sparse_map_msgs::SaveMap::Request &req,
                              sparse_map_msgs::SaveMap::Response &res)
{
        //Writes as ascii pcd file
        std::string filename = req.filename.c_str();
        std::ofstream file(filename);
        if(!file.is_open())
        {
                std::cout << "Error writing to file file:" << filename <<'\n';
                res.success = false;
                return true;
        }
        //Write pcd header
        file << "# Quantization nodes!\n";
        file << "VERSION .7\n";
        file << "FIELDS x y z label\n";
        file << "SIZE 4 4 4 4\n";
        file << "TYPE F F F I\n";
        file << "COUNT 1 1 1 1\n";
        file << "WIDTH " << freeCodebook.size() + occCodebook.size() << std::endl;
        file << "HEIGHT 1\n";
        file << "VIEWPOINT 0 0 0 1 0 0 0\n";
        file << "POINTS " << freeCodebook.size() + occCodebook.size()  << std::endl;
        file << "DATA ascii\n";
        //DATA
        for (pointGeom const &p : occCodebook) {
                file<<p.x << " " << p.y << " " << p.z << " 0" << std::endl;
        }

        for (pointGeom const &p : freeCodebook) {
                file<<p.x << " " << p.y << " " << p.z << " 1" << std::endl;
        }

        file.close();
        std::cout << "File saved successfully" << '\n';
        res.success = true;
        return true;
}
double sparseMapper::normL1(pointGeom p)
{
        return p.x*p.x+p.y*p.y+p.z*p.z;
}

// pointGeom::pointGeom(geometry_msgs::Point p)
// {
//         this->x=p.x;
//         this->y=p.y;
//         this->z=p.z;
// }
//
//
// bool pointGeom::operator<(const pointGeom & p) const
// {
//         return p.normL1()< this.normL1();
// }
