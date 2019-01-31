#include "sparse_map_server/sparse_map_editor.h"

sparseMapEditor::sparseMapEditor(ros::NodeHandle &nh)
{
        nh_=nh;
        std::string graphFileName;
        ros::NodeHandle nh_priv("~");
        nh_priv.param<std::string>("graph_file",graphFileName,"sparse_map.txt");
        nh_priv.param<std::string>("map_frame",mapFrame,"map");
        nh_priv.param<std::string>("mode",opMode,"remove");
        nh_priv.param<std::string>("set",opSet,"occupied");
        float pruneMax,pruneMin;
        nh_priv.param<float>("nodes_max_z",pruneMax,0.0);
        nh_priv.param<float>("nodes_min_z",pruneMin,0.0);
        codesPublisher = nh_.advertise<visualization_msgs::Marker>("centroids_marker",1,true);
        removeSub = nh_.subscribe("clicked_point",1,&sparseMapEditor::modifyCodes,this);
        savingSrv = nh_.advertiseService("save_map",&sparseMapEditor::saveNodes,this);
        updateSrv = nh_.advertiseService("update_params",&sparseMapEditor::updateParameters,this);

        std::cout << "Starting with mode " << opMode << " on set " << opSet << '\n';
        codes.loadNodes(graphFileName);
        bool doPassThrough = ( (abs(pruneMax) > 0) ||  (abs(pruneMin) >0) );
        if (doPassThrough) {
                int nPruned = codes.simpleZPassThrough(pruneMax,pruneMin);
                std::cout << "Removed: " << nPruned << " nodes \n";

        }
        pointArray occCodes = codes.getOccCodes();
        pointArray freeCodes = codes.getFreeCodes();
        std::cout << occCodes.size() << "free nodes \n";
        std::cout << freeCodes.size() << "occupied nodes \n";
        std_msgs::ColorRGBA colorFree;
        colorFree.r=0.5;   colorFree.g=1.0;   colorFree.b=0.5;   colorFree.a=1.0;
        std_msgs::ColorRGBA colorOcc;
        colorOcc.r=1.0;   colorOcc.g=0.0;   colorOcc.b=0.5;
        colorOcc.a=1.0;
        makeCodesMarkerAndPublish(occCodes,colorOcc,0);
        makeCodesMarkerAndPublish(freeCodes,colorFree,1);
        return;
}

bool sparseMapEditor::updateParameters(std_srvs::Empty::Request& request,
                                       std_srvs::Empty::Response& response )
{
        ros::NodeHandle nh_priv("~");

        nh_priv.param<std::string>("map_frame",mapFrame,"map");
        nh_priv.param<std::string>("mode",opMode,"remove");
        nh_priv.param<std::string>("set",opSet,"occupied");
        std::cout << "Changing to mode " << opMode << " on set " << opSet << '\n';

        return true;
}

void sparseMapEditor::modifyCodes(const geometry_msgs::PointStamped &msg)
{

        if (!opMode.compare("remove")) {
                //Removes point cicled from rviz
                pointGeom removedPoint;
                if (!opSet.compare("occupied"))
                {
                        removedPoint = codes.removeOccCode(msg.point);
                }
                else if (!opSet.compare("free"))
                {
                        removedPoint = codes.removeFreeCode(msg.point);
                }
                else
                {
                        std::cout << "Invalid space" <<  opSet<<'\n';
                        return;
                }
                std::cout << "Point "<< removedPoint <<" removed\n";
        }
        else if (!opMode.compare("add"))
        {
                if (!opSet.compare("occupied"))
                {
                        codes.addOccCode(msg.point);
                }
                else if (!opSet.compare("free"))
                {
                        codes.addFreeCode(msg.point);
                }
                else
                {
                        std::cout << "Invalid space" <<  opSet<<'\n';
                        return;
                }
                std::cout << "Point "<< msg.point <<" added\n";
        }
        else
        {
                std::cout << "Error on parameters" << opMode << opSet<< '\n';
        }


        std_msgs::ColorRGBA colorFree;
        colorFree.r=0.5;   colorFree.g=1.0;   colorFree.b=0.5;   colorFree.a=1.0;
        std_msgs::ColorRGBA colorOcc;
        colorOcc.r=1.0;   colorOcc.g=0.0;   colorOcc.b=0.5;
        colorOcc.a=1.0;
        pointArray occ = codes.getOccCodes();
        pointArray libre = codes.getFreeCodes();
        makeCodesMarkerAndPublish(occ,colorOcc,0);
        makeCodesMarkerAndPublish(libre,colorFree,1);
        return;
}

void sparseMapEditor::makeCodesMarkerAndPublish(pointArray &codes, std_msgs::ColorRGBA color, int id)
{
        const float radius = 0.05;
        visualization_msgs::Marker centroidsMarker;
        //centroidsMarker.points.resize(codebook.size());
        centroidsMarker.ns = "centroids_static";
        centroidsMarker.action = visualization_msgs::Marker::ADD;
        centroidsMarker.header.frame_id = mapFrame;
        centroidsMarker.header.stamp = ros::Time();
        centroidsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        centroidsMarker.pose.orientation.w = 1.0;
        centroidsMarker.scale.x = radius;
        centroidsMarker.scale.y = radius;
        centroidsMarker.scale.z = radius;
        centroidsMarker.id = id;
        centroidsMarker.color = color;

        centroidsMarker.points = codes;
        codesPublisher.publish(centroidsMarker);
}

bool sparseMapEditor::saveNodes(sparse_map_msgs::SaveMap::Request &req,
                                sparse_map_msgs::SaveMap::Response &res)
{
        res.success = codes.saveAsTxt(req.filename);
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
