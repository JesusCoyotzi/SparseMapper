#include "space_separation.h"
spaceSeparator::spaceSeparator(ros::NodeHandle &nh)
{
        nh_=nh;
        ros::NodeHandle nh_priv("~");

        nh_priv.param<float>("free_thr",freeThr,0.1);
        nh_priv.param<bool>("pub_seg_space",pubSegSpace,true);
}

void spaceSeparator::setVQMapCallbacks()
{
        //used space quantization to generate free and occupied space
        // freeCloudPub = nh_.advertise<sensor_msgs::PointCloud2>("free_space",2);
        // occCloudPub = nh_.advertise<sensor_msgs::PointCloud2>("occupied_space",2);
        // quantizedSpacePub= nh_.advertise<space_quantization::quantizedSpace>("occupied_quantized",2);
        // spaceSub = nh_.subscribe("quantized_space",10, &spaceSeparator::spaceCallback, this);
        //Now this node only does visualization
        markerPub = nh_.advertise<visualization_msgs::MarkerArray>("centroids_marker_array",2);
        codebookSub = nh_.subscribe("codebook",10, &spaceSeparator::spaceCallback, this);
        return;
}

void spaceSeparator::setVoxelMapCallbacks()
{
        //Uses octomap as a voxelization technique.
        //subscribe to pointcloud2 callback and separate space
        octoSub = nh_.subscribe("voxelmap_cloud",10, &spaceSeparator::voxelMapCloudCallback,this);
        freeCodebookPub = nh_.advertise<space_quantization::codebook>("free_voxels",2);
        occupiedCodebookPub = nh_.advertise<space_quantization::codebook>("occupied_voxels",2);
        markerPub = nh_.advertise<visualization_msgs::Marker>("voxels_marker",2);
        return;
}



void spaceSeparator::voxelSeparatorAndPublish(  Point3 * Ps, int nP, pointArray &codebookFree, pointArray &codebookOcc)
{
        ///Check all points check if z component islower than threes and
        for (size_t i = 0; i < nP; i++) {
                geometry_msgs::Point p;
                p.x=Ps[i].x;
                p.y=Ps[i].y;
                p.z=Ps[i].z;
                if (Ps[i].z<=freeThr) {
                        codebookFree.push_back(p);
                }
                else {
                        codebookOcc.push_back(p);
                }
        }
        space_quantization::codebook c;
        c.header.frame_id=voxelCloudFrame;
        c.header.stamp=voxelStamp;
        c.centroids = codebookFree;
        freeCodebookPub.publish(c);
        c.centroids = codebookOcc;
        occupiedCodebookPub.publish(c);
        return;
}

void spaceSeparator::voxelMapCloudCallback(const sensor_msgs::PointCloud2 &msg)
{
        //gET PointCloud2 check all voxels and threhold heaight to send
        //Ground as free space
        //unpacking
        sensor_msgs::PointCloud2 space = msg;
        int n=space.height*space.width;
        std::cout << "Got label cloud of:" << n << "Points"<< '\n';
        voxelCloudFrame = space.header.frame_id;
        voxelStamp = space.header.stamp;
        std::cout << "In frame " << voxelCloudFrame << " on Stamp " <<voxelStamp<< '\n';

        //extract pointcloud to a more convenient data storage

        Point3 * Points = (Point3 *)malloc(n*sizeof(Point3));
        int validP=toPoint3(space,Points);
        std::cout << "ValidP: " << validP<<'\n';
        if (validP<1)
        {
                ROS_ERROR("No points read");
                return;
        }
        pointArray codebookF, codebookO;
        voxelSeparatorAndPublish(Points,validP,codebookF,codebookO);
        makeVizMarkerAndPublish(codebookF,true);
        makeVizMarkerAndPublish(codebookO,false);

        free(Points);
}

void spaceSeparator::spaceCallback(const space_quantization::codebook &msg)
{
        //unpacking
        std::cout << "Got codebook of:" << msg.centroids.size() << "Points"<< '\n';
        cloudFrame = msg.header.frame_id;
        stamp = msg.header.stamp;
        std::cout << "In frame " << cloudFrame << "on Stamp " <<stamp<< '\n';
        std::vector<geometry_msgs::Point> codebook = msg.centroids;
        //extract pointcloud to a more convenient data storage
        //separate in free and occupied space and publish all.
        //Both for visualization and processing
        //separateSpaceAndPublish(labeledPoints, codebook, n);
        //voxelSeparatorAndPublish(Points,validP,codebookF,codebookO);
        makeVizMsgAndPublish(codebook);
        return;
}

float spaceSeparator::makeFloat(unsigned char * byteArray)
{
        charToFloat S;
        for (size_t i = 0; i <4; i++)
        {
                S.byteStream[i]=byteArray[i];
        }

        return S.assembledFloat;
}

int spaceSeparator::makeInt(unsigned char * byteArray)
{
        charToFloat S;
        for (size_t i = 0; i <4; i++)
        {
                S.byteStream[i]=byteArray[i];
        }
        return S.assembledInt;
}

int spaceSeparator::toPoint3(sensor_msgs::PointCloud2 Cloud,
                             Point3 * points)
{
        /*cloud cloud on the robot base frame;
           points arrray of points to store valid points,
           returns: Number of no NaN poinst actually stored, continously on points         */
        int point_step = Cloud.point_step;
        int data_step = sizeof(float);
        std::cout << "data_step: " << data_step <<'\n';
        std::vector<unsigned char> byteArray = Cloud.data;
        int j=0;
        for (size_t i = 0; i < byteArray.size(); i+=point_step)
        {

                float tmp =makeFloat(&byteArray[i]);
                if(!std::isnan(tmp)) {
                        points[j].x=makeFloat(&byteArray[i]);
                        points[j].y=makeFloat(&byteArray[i+data_step]);
                        points[j].z=makeFloat(&byteArray[i+2*data_step]);
                        j++;
                }
        }
        // std::cout << "Point " << points[j/2].x <<" "
        //           <<    points[j/2].y <<" "
        //           <<    points[j/2].z << '\n';
        // std::cout << j << '\n';
        return j;
}

int spaceSeparator::tolabelPoint3(sensor_msgs::PointCloud2 Cloud,
                                  labelPoint3 * points)
{
        /*cloud cloud on the robot base frame;
           points arrray of points to store valid points,
           returns: Number of no NaN poinst actually stored, continously on points         */
        int point_step = Cloud.point_step;
        int data_step = sizeof(float);
        std::cout << "data_step: " << data_step <<'\n';
        std::vector<unsigned char> byteArray = Cloud.data;
        int j=0;
        for (size_t i = 0; i < byteArray.size(); i+=point_step)
        {
                //std::cout << "WHYYY!!!!" << '\n';
                float tmp =makeFloat(&byteArray[i]);
                if(!std::isnan(tmp)) {
                        points[j].x=makeFloat(&byteArray[i]);
                        points[j].y=makeFloat(&byteArray[i+data_step]);
                        points[j].z=makeFloat(&byteArray[i+2*data_step]);
                        points[j].label = makeInt(&byteArray[i+3*data_step]);
                        j++;
                        //std::cout << j << '\n';
                }
        }
        // std::cout << "Point" << points[j/2].x <<" "
        //           <<    points[j/2].y <<" "
        //           <<    points[j/2].z <<" "
        //           <<    points[j/2].label<< '\n';

        return j;
}

void spaceSeparator::makeCloudHeader(sensor_msgs::PointCloud2 &cloud, int points)
{
        //Simple function, makes the header and field of a message
        //Create a pointcloud with a int32 message
        //wich includes the label
        //Finally : Fields: x ,y ,z , label
        cloud.header.frame_id = cloudFrame;
        cloud.header.stamp = stamp; //**//
        cloud.width = points;
        cloud.height = 1;
        cloud.is_bigendian = true;
        cloud.point_step = 4*4; //3 floats, x y z and one int label of 4 bytes each
        cloud.row_step = points*cloud.point_step;

        //Fields for the cloud
        sensor_msgs::PointField pf;
        pf.name = "x";  pf.offset = 0;  pf.datatype = (unsigned char)7; pf.count =1;
        cloud.fields.push_back(pf);
        pf.name = "y";  pf.offset = 4;  pf.datatype = (unsigned char)7; pf.count =1;
        cloud.fields.push_back(pf);
        pf.name = "z";  pf.offset = 8;  pf.datatype = (unsigned char)7; pf.count =1;
        cloud.fields.push_back(pf);
        pf.name = "label";  pf.offset = 12;  pf.datatype = (unsigned char)5; pf.count =1;
        cloud.fields.push_back(pf);
        return;
}

void
spaceSeparator::separateSpaceAndPublish(labelPoint3* space,
                                        pointArray codebook,
                                        int nPoints)
{
        // Ensemble pointcloud for free & occupied space.
        // and vomit them on to a fancy cloud2 msg.
        //Count free points and occupied points
        int nFreePoints=0, nOccPoints =0;
        for (unsigned int i = 0; i < nPoints; i++)
        {
                float cntHeight = codebook[space[i].label].z;
                if (cntHeight<=freeThr) {
                        nFreePoints++;
                }
        }
        nOccPoints =nPoints-nFreePoints;
        // set info for Cloud.
        sensor_msgs::PointCloud2 freeCloud,occCloud;
        makeCloudHeader(freeCloud,nFreePoints);
        makeCloudHeader(occCloud,nOccPoints);
        //Now ensemble the byteBlob
        //4 float of 4 bytes of nFreePoints 4*4*nFreePoints
        std::vector<unsigned char> byteBlobFree(4*4*nFreePoints);
        std::vector<unsigned char> byteBlobOcc(4*4*nOccPoints);
        charToFloat changer;
        //This could be easily the most complex for in my
        //whoole life
        for (int j=0, i=0, w=0; j< nPoints; j++)
        {
                std::vector<unsigned char> tmp(4*4);

                changer.assembledFloat = space[j].x;
                int u,k;
                for (u = 0,k = 0; k < 4; k++, u++) {
                        tmp[u]=changer.byteStream[k];
                }
                changer.assembledFloat = space[j].y;
                for (k = 0; k < 4; k++, u++) {
                        tmp[u]=changer.byteStream[k];
                }
                changer.assembledFloat = space[j].z;
                for (k = 0; k < 4; k++, u++) {
                        tmp[u]=changer.byteStream[k];
                }
                changer.assembledInt = space[j].label;
                for (k = 0; k < 4; k++, u++) {
                        tmp[u]=changer.byteStream[k];
                }
                // for (unsigned int v = 0; v < 4*4; v++) {
                //
                //         byteBlobLabel[i+v]=tmp[v];
                // }
                // i+=labeledCloud.point_step;
                float cntHeight = codebook[space[j].label].z;
                if (cntHeight<=freeThr)
                {
                        for (int v = 0; v < 16; v++) {
                                byteBlobFree[i+v]=tmp[v];
                        }
                        i+=freeCloud.point_step;
                }
                else
                {
                        for (int v = 0; v < 16; v++) {
                                byteBlobOcc[w+v]=tmp[v];
                        }
                        w+=occCloud.point_step;
                }

        }
        freeCloud.data = byteBlobFree;
        occCloud.data = byteBlobOcc;


        //make quantizedSpace obj
        space_quantization::quantizedSpace qs;
        qs.space = occCloud;
        //Pass only free space centroids
        for (size_t i = 0; i < codebook.size(); i++) {
                if (codebook[i].z<=freeThr) {
                        qs.codebook.push_back(codebook[i]);
                }
        }
        //Allows for publish only segmented cloud
        if (pubSegSpace)
        {
                freeCloudPub.publish(freeCloud);
                occCloudPub.publish(occCloud);
        }
        //Publish occ space + centroids
        quantizedSpacePub.publish(qs);
        return;
}



void spaceSeparator::makeVizMsgAndPublish( pointArray &codebook)
{
        const float radius = 0.05;
        visualization_msgs::MarkerArray centroidsArray;
        centroidsArray.markers.resize(codebook.size());
        for (int i = 0; i < codebook.size(); i++) {
                centroidsArray.markers[i].ns = "centroids";
                centroidsArray.markers[i].id = i;
                centroidsArray.markers[i].action = visualization_msgs::Marker::ADD;
                centroidsArray.markers[i].header.frame_id = cloudFrame;
                centroidsArray.markers[i].header.stamp = ros::Time();
                centroidsArray.markers[i].type = visualization_msgs::Marker::SPHERE;
                centroidsArray.markers[i].pose.position.x = codebook[i].x;
                centroidsArray.markers[i].pose.position.y = codebook[i].y;
                centroidsArray.markers[i].pose.position.z = codebook[i].z;
                centroidsArray.markers[i].pose.orientation.x = 0.0;
                centroidsArray.markers[i].pose.orientation.y = 0.0;
                centroidsArray.markers[i].pose.orientation.z = 0.0;
                centroidsArray.markers[i].pose.orientation.w = 1.0;
                centroidsArray.markers[i].scale.x = radius;
                centroidsArray.markers[i].scale.y = radius;
                centroidsArray.markers[i].scale.z = radius;
                centroidsArray.markers[i].color.a = 1.0;
                centroidsArray.markers[i].lifetime = ros::Duration(0);

                if (codebook[i].z<=freeThr) {
                        centroidsArray.markers[i].color.g = 1.0;
                }
                else {
                        centroidsArray.markers[i].color.r = 1.0;
                }

        }
        markerPub.publish(centroidsArray);
        return;
}

void spaceSeparator::makeVizMarkerAndPublish( pointArray &codebook, bool free)
{
        const float radius = 0.05;
        visualization_msgs::Marker centroidsMarker;
        //centroidsMarker.points.resize(codebook.size());
        centroidsMarker.ns = "centroids";

        centroidsMarker.action = visualization_msgs::Marker::ADD;
        centroidsMarker.header.frame_id = voxelCloudFrame;
        centroidsMarker.header.stamp = ros::Time();
        centroidsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        centroidsMarker.pose.orientation.w = 1.0;
        centroidsMarker.scale.x = radius;
        centroidsMarker.scale.y = radius;
        centroidsMarker.scale.z = radius;

        if (free) {
                centroidsMarker.id = 1;
                centroidsMarker.color.r=0.0;
                centroidsMarker.color.b=1.0;
                centroidsMarker.color.g=0.0;
                centroidsMarker.color.a=1.0;

        }
        else
        {
                centroidsMarker.id = 2;
                centroidsMarker.color.r=1.0;
                centroidsMarker.color.b=0.0;
                centroidsMarker.color.g=0.0;
                centroidsMarker.color.a=1.0;
        }
        centroidsMarker.points = codebook;
        std::cout << "Publishing coddeob of:" << codebook.size()<<'\n';
        markerPub.publish(centroidsMarker);
        return;
}
