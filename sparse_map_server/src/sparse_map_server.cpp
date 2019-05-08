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
        nh_priv.param<float>("max_dist_terminal",maxDistTerm,1.5);
        nh_priv.param<float>("min_dist",minDist,1);
        nh_priv.param<int>("k_neighboors",kNeighboors,6);
        nh_priv.param<bool>("visualize_nodes",visNodes,true);
        //nh_priv.param<bool>("visualize_path",visPath,true);
        nh_priv.param<bool>("visualize_terminals",visTerminals,true);
        nh_priv.param<bool>("validate_terminals",validateTerminals,true);

        codebookMarkerPub= nh_.advertise<visualization_msgs::Marker>("centroids_marker",1,true);
        graphMarkerPub= nh_.advertise<visualization_msgs::Marker>("graph_marker",1,true);
        labelPub = nh_.advertise<visualization_msgs::MarkerArray>("label_marker",1,true);
        terminalPub  = nh_.advertise<visualization_msgs::MarkerArray>("terminal_marker",1,true);
        voxelPub= nh_.advertise<visualization_msgs::Marker>("voxel_marker",1,true);

        pathServer = nh_.advertiseService("make_plan",&sparseMapServer::getPlan,this);
        graphSaverServer = nh_.advertiseService("save_graph",&sparseMapServer::saveGraph,this);

        pathPub=nh_.advertise<nav_msgs::Path>("sparse_plan",1);

        rebuildPub = nh_.subscribe("remake_graph",1,&sparseMapServer::remakeGraph,this);
        //rebuildPub = nh_.subscribe("clicked_point",1,&sparseMapServer::removePoint,this);

        //If desired can provide precomputed nodes and edges file
        nh_priv.param<bool>("use_existing_graph",useExisting,false);
        if (useExisting) {
                nh_priv.param<std::string>("nodes_file",nodeFile,"nodes.pcd");
                nh_priv.param<std::string>("edges_file",edgeFile,"edges.grph");
                sparseMap = adjacencyMap(nodeFile,edgeFile,safetyHeight,safetyRadius,maxDistTerm);
        }
        else
        {
                sparseMap = adjacencyMap(mapFileName,safetyHeight,safetyRadius,
                                         connectionRadius,
                                         maxDist,minDist,maxDistTerm,kNeighboors);
                sparseMap.makeGraph();
        }


        //Voxel Grid
        occupiedGrid.setStep(safetyRadius*2); //for diameter
        occupiedGrid.voxelize(sparseMap.getOccNodes());
        //occupiedGrid.printVoxGrid();
        //To let advertisers enable
        ros::Duration(1).sleep();
        if(visNodes)
        {
                std_msgs::ColorRGBA freeColor = makeColor(0.5,1.0,0.50,1.0);
                std_msgs::ColorRGBA occColor  = makeColor(1,0.0,0.5,1.0);
                pointArray occ = sparseMap.getOccNodes();
                pointArray libre = sparseMap.getFreeNodes();
                adjacencyList adjL = sparseMap.getEdges();
                makeCentroidsMarkerAndPublish(occ,occColor,0);
                makeCentroidsMarkerAndPublish(libre,freeColor,1);
                makeVizGraphAndPublish(adjL,libre);
                makeLabelMsgAndPublish(libre,2); //second params is id
        }
        return;
}

void sparseMapServer::removePoint(const geometry_msgs::PointStamped &msg)
{
        //Removes point clicled from rvbiz
        if (sparseMap.removeOccupiedPoint(msg.point))
        {
                std::cout << "Point removed" << '\n';
                //sparseMap.makeGraph();
                if(visNodes)
                {
                        std_msgs::ColorRGBA freeColor = makeColor(0.5,1.0,0.50,1.0);
                        std_msgs::ColorRGBA occColor  = makeColor(1,0.0,0.5,1.0);
                        pointArray occ = sparseMap.getOccNodes();
                        pointArray libre = sparseMap.getFreeNodes();
                        makeCentroidsMarkerAndPublish(occ,occColor,0);
                        makeCentroidsMarkerAndPublish(libre,freeColor,1);
                }
        }
        return;
}

bool sparseMapServer::saveGraph(sparse_map_msgs::SaveMap::Request &req,
                                sparse_map_msgs::SaveMap::Response &res)
{
        graphIO graphSaver;
        graphSaver.loadMap(sparseMap);
        res.success = graphSaver.saveGraph(req.filename);
        return true;
}

void sparseMapServer::makeTerminalsAndPublish(pointGeom start, pointGeom goal)
{
        visualization_msgs::MarkerArray terminals;
        visualization_msgs::Marker startMarker;
        startMarker.ns = "labels_static";
        startMarker.action = visualization_msgs::Marker::ADD;
        startMarker.header.frame_id = mapFrame;
        startMarker.header.stamp = ros::Time();
        startMarker.type = visualization_msgs::Marker::CYLINDER;
        startMarker.pose.orientation.w = 1.0;
        startMarker.scale.z = safetyHeight;
        startMarker.scale.x = safetyRadius*2;
        startMarker.scale.y = safetyRadius*2;
        startMarker.id =0;
        startMarker.pose.position = start;
        startMarker.pose.position.z += safetyHeight/2;
        startMarker.color.r = 0.0f;
        startMarker.color.g = 0.0f;
        startMarker.color.b = 1.0f;
        startMarker.color.a = 1.0;
        terminals.markers.push_back(startMarker);

        visualization_msgs::Marker goalMarker;
        goalMarker.ns = "labels_static";
        goalMarker.action = visualization_msgs::Marker::ADD;
        goalMarker.header.frame_id = mapFrame;
        goalMarker.header.stamp = ros::Time();
        goalMarker.type = visualization_msgs::Marker::CYLINDER;
        goalMarker.pose.orientation.w = 1.0;
        goalMarker.scale.z = safetyHeight;
        goalMarker.scale.x = safetyRadius*2;
        goalMarker.scale.y = safetyRadius*2;
        goalMarker.id = 1;
        goalMarker.pose.position = goal;
        goalMarker.pose.position.z += safetyHeight/2;
        goalMarker.color.r = 0.976f;
        goalMarker.color.g = 0.451f;
        goalMarker.color.b = 0.024f;
        goalMarker.color.a = 1.0;
        terminals.markers.push_back(goalMarker);

        terminalPub.publish(terminals);

        return;

}

void sparseMapServer::makeLabelMsgAndPublish(pointArray &codebook,int id)
{
        const float radius = 0.1;
        visualization_msgs::MarkerArray labels;
        for (size_t i = 0; i < codebook.size(); i++)
        {
                visualization_msgs::Marker labelMarker;
                labelMarker.ns = "labels_static";
                labelMarker.action = visualization_msgs::Marker::ADD;
                labelMarker.header.frame_id = mapFrame;
                labelMarker.header.stamp = ros::Time();
                labelMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                labelMarker.pose.orientation.w = 1.0;
                labelMarker.scale.z = radius;
                labelMarker.scale.x = radius;
                labelMarker.scale.y = radius;
                labelMarker.id =i;
                labelMarker.pose.position = codebook[i];
                labelMarker.pose.position.z+=0.1;
                labelMarker.text=std::to_string(i);
                labelMarker.color.r = 0.0f;
                labelMarker.color.g =1.0f;
                labelMarker.color.b = 0.0f;
                labelMarker.color.a = 1.0;
                labels.markers.push_back(labelMarker);
        }
        std::cout << "Publishing " << labels.markers.size() <<'\n';
        labelPub.publish(labels);
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
        connections.scale.x = 0.010;
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

void sparseMapServer::makeVoxelsAndPublish( pointArray voxels, float size)
{
        visualization_msgs::Marker voxelsMarker;
        voxelsMarker.header.frame_id=mapFrame;
        voxelsMarker.header.stamp=ros::Time::now();
        voxelsMarker.ns = "voxelsMarker_static";
        voxelsMarker.action = visualization_msgs::Marker::ADD;
        voxelsMarker.pose.orientation.w = 1.0;
        voxelsMarker.id = 4;
        voxelsMarker.type = visualization_msgs::Marker::CUBE_LIST;
        voxelsMarker.scale.x = size;
        voxelsMarker.scale.y = size;
        voxelsMarker.scale.z = size;
        voxelsMarker.color.r=1.0;
        voxelsMarker.color.b=0.0;
        voxelsMarker.color.g=1.0;
        voxelsMarker.color.a=0.75;
        voxelsMarker.points=voxels;
        voxelPub.publish(voxelsMarker);
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
        //std::cout << "Calculating path" << '\n';
        pointArray pth;
        if (visTerminals) {
                makeTerminalsAndPublish(req.goalPose,req.startPose);
        }

        //Check if goal and start are valid
        //  ros::Time validStart = ros::Time::now();
        int start, goal;
        bool isValid;
        if (validateTerminals)
        {
                std::cout << "Using full validation" << '\n';
                pointArray checkingCodes = occupiedGrid.getPointsInVoxel(req.startPose);
                bool isValidStrt = sparseMap.validateSingleTerminal(req.startPose,start,checkingCodes);
                checkingCodes = occupiedGrid.getPointsInVoxel(req.goalPose);
                bool isValidGoal = sparseMap.validateSingleTerminal(req.goalPose,goal,checkingCodes);
                isValid = isValidStrt && isValidGoal;

                //To debug voxel grid
                // pointArray voxelCenters = occupiedGrid.getVoxelCentroids(req.goalPose);
                // makeVoxelsAndPublish(voxelCenters,safetyRadius*2);

        }
        else
        {
                //std::cout << "Using Quickq validation" << '\n';
                isValid = sparseMap.validateTerminalsQuick(req.startPose,req.goalPose,start,goal);
        }
        //ros::Duration validRun = ros::Time::now() - validStart;

        res.plan.header.frame_id = mapFrame;
        res.plan.header.stamp = ros::Time();
        if (!isValid) {
                std::cout <<
                " \033[1;31m Error either initial or final position collides, with occupied nodes or is too far \033[0m \n";
                return true;
        }

        //ros::Time planStart = ros::Time::now();
        if(!sparseMap.Astar(goal,start,pth))
        {
                //failure
                return true;
        }

        geometry_msgs::Quaternion q;
        q.x=0; q.y=0; q.z=0; q.w=1;
        geometry_msgs::PoseStamped local_pose;
        local_pose.pose.position = req.startPose;
        local_pose.pose.orientation = q;

        res.plan.poses.push_back(local_pose);

        for (size_t i = 0; i < pth.size(); i++) {
                local_pose.pose.position = pth[i];
                res.plan.poses.push_back(local_pose);
                // std::cout << pth[i].x << " "
                //           << pth[i].y << " "
                //           << pth[i].z << '\n';
        }
        local_pose.pose.position = req.goalPose;
        res.plan.poses.push_back(local_pose);
        //ros::Duration planRun = ros::Time::now() - planStart;

        std::cout << "Path planning succesfull: Nodes visited: " << pth.size()+2 <<'\n';
        // std::cout << "Validation time: " << validRun << '\n';
        // std::cout << "Planning time: " << planRun << '\n';
        pathPub.publish(res.plan);

        return true;
}


void sparseMapServer::remakeGraph(const std_msgs::Empty &msg)
{
        //call function to remake graph based on params
        //WARNING this functions has not yet been tested and online recalculation
        // Of the graph is still not supported
        std::cout << "!!!Rebuilding graph!!!" << '\n';
        sparseMap.makeGraph();

        pointArray occ = sparseMap.getOccNodes();
        pointArray libre = sparseMap.getFreeNodes();
        adjacencyList adjL = sparseMap.getEdges();
        std_msgs::ColorRGBA freeColor = makeColor(0.5,1.0,0.50,1.0);
        std_msgs::ColorRGBA occColor  = makeColor(1,0.0,0.5,1.0);
        makeCentroidsMarkerAndPublish(occ,occColor,0);
        makeCentroidsMarkerAndPublish(libre,freeColor,1);
        makeVizGraphAndPublish(adjL,libre);
        return;
}
