#include "sparse_mapper/sparse_mapper.h"

sparseMapper::sparseMapper(ros::NodeHandle &nh)
{
        nh_=nh;
        ros::NodeHandle nh_priv("~");
        nh_priv.param<float>("free_thr",freeThr,0.1);
        // nh_priv.param<std::string>("graph_file",graphFile,"adjGraph.txt");

        codebookSub = nh_.subscribe("codebook",10, &sparseMapper::codebookCallback, this);
        graphMakeSub = nh_.advertiseService("make_graph", &sparseMapper::saveMapSrv,this);
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
