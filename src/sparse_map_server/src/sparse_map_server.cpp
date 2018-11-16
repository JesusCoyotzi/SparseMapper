#include "sparse_map_server/sparse_map_server.h"

sparseMapServer::sparseMapServer(ros::NodeHandle &nh)
{
        std::cout << "Setting subscribers and publishers" << '\n';
        nh_=nh;

        ros::NodeHandle nh_priv("~");
        nh_priv.param<std::string>("map_file",mapFileName,"sparse_map.txt");
        nh_priv.param<std::string>("map_frame",mapFrame,"map");
        nh_priv.param<float>("safety_height",safetyHeight,1.2);
        nh_priv.param<float>("safety_radius",safetyRadius,0.75);
        nh_priv.param<float>("connection_radius",connectionRadius,0.4);
        nh_priv.param<float>("max_dist",maxDist,2);
        nh_priv.param<int>("k_neighboors",kNeighboors,6);

        codebookMarkerPub= nh_.advertise<visualization_msgs::Marker>("centroids_marker",1,true);
        graphMarkerPub= nh_.advertise<visualization_msgs::Marker>("graph_marker",1,true);

        pathServer = nh_.advertiseService("make_plan",&sparseMapServer::getPlan,this);

        pathPub=nh_.advertise<nav_msgs::Path>("sparse_plan",1);

        sparseMap = adjacencyMap(mapFileName);
        std_msgs::ColorRGBA freeColor = makeColor(0.5,1.0,0.50,1.0);
        std_msgs::ColorRGBA occColor  = makeColor(1,0.0,0.5,1.0);

        pointArray occ = sparseMap.getOccNodes();
        pointArray libre = sparseMap.getFreeNodes();
        adjacencyList adjL = sparseMap.getGraph();
        //To let advertisers enable
        ros::Duration(1).sleep();
        makeCentroidsMarkerAndPublish(occ,occColor,0);
        makeCentroidsMarkerAndPublish(libre,freeColor,1);
        makeVizGraphAndPublish(adjL,libre);
        return;
}


void sparseMapServer::makeCentroidsMarkerAndPublish(pointArray &codebook, std_msgs::ColorRGBA color,int id)
{
        const float radius = 0.05;
        visualization_msgs::Marker centroidsMarker;
        //centroidsMarker.points.resize(codebook.size());
        centroidsMarker.ns = "centroids_static";
        centroidsMarker.action = visualization_msgs::Marker::ADD;
        centroidsMarker.header.frame_id = mapFrame;
        centroidsMarker.header.stamp = ros::Time();
        centroidsMarker.type = visualization_msgs::Marker::CUBE_LIST;
        centroidsMarker.pose.orientation.w = 1.0;
        centroidsMarker.scale.x = radius;
        centroidsMarker.scale.y = radius;
        centroidsMarker.scale.z = radius;
        centroidsMarker.id =id;
        centroidsMarker.color = color;
        std::cout << "publish codebook of " <<  codebook.size()<<'\n';
        centroidsMarker.points = codebook;
        codebookMarkerPub.publish(centroidsMarker);
        return;
}

void sparseMapServer::makeVizGraphAndPublish(adjacencyList l, pointArray codebook)
{
        visualization_msgs::Marker connections;
        connections.header.frame_id=mapFrame;
        connections.header.stamp=ros::Time::now();
        connections.ns = "grafo";
        connections.pose.orientation.w = 1.0;
        connections.id = 3;
        connections.type = visualization_msgs::Marker::LINE_LIST;
        connections.scale.x = 0.025;
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
                        connections.points.push_back(codebook[i]);
                        connections.points.push_back(codebook[k]);
                }
        }
        graphMarkerPub.publish(connections);
}

std_msgs::ColorRGBA sparseMapServer::makeColor(float r,float g, float b, float a)
{
        std_msgs::ColorRGBA color;
        color.r=r;
        color.g=g;
        color.b=b;
        color.a=a;
        return color;
}

bool sparseMapServer::getPlan(sparse_map_msgs::MakePlan::Request &req,
                              sparse_map_msgs::MakePlan::Response &res)
{
        std::cout << "Calculating path" << '\n';
        pointArray pth = sparseMap.Astar(req.goalPose,req.startPose);
        res.path.header.frame_id = mapFrame;
        res.path.header.stamp = ros::Time();
        std::cout << "Nodes visited: " << '\n';
        for (size_t i = 0; i < pth.size(); i++) {
                geometry_msgs::PoseStamped local_pose;
                local_pose.pose.position = pth[i];
                local_pose.pose.orientation.x = 0;
                local_pose.pose.orientation.y = 0;
                local_pose.pose.orientation.z = 0;
                local_pose.pose.orientation.w = 1;
                res.path.poses.push_back(local_pose);
                std::cout << pth[i].x << " "
                          << pth[i].y << " "
                          << pth[i].z << '\n';
        }
        pathPub.publish(res.path);
        return true;
}
