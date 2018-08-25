#include "space_seg.h"

spaceSegmenter::spaceSegmenter(ros::NodeHandle nh)
{
        ros::NodeHandle nh_priv("~");
        nh_priv.param<int>("nClusters",nClusters,8);
        nh_priv.param<int>("iterations",iterations,16);
        nh_priv.param<bool>("visualizeCentroids",vizCnt,false);
        nh_priv.param<bool>("visualizeSegCloud",vizSegCloud,true);
        nh_priv.param<float>("freeThr",freeThr,0.1);
        nh_priv.param<std::string>("baseFrame",baseFrame,"base_link");
        nh_priv.param<std::string>("cloudFrame",cloudFrame,
                                   "head_rgbd_sensor_rgb_frame");

        sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>
                            ("cloud",10,
                            &spaceSegmenter::cloudCallback,
                            this);
        freeCloud_pub = nh.advertise<sensor_msgs::PointCloud2>
                                ("free_space",2);
        occCloud_pub = nh.advertise<sensor_msgs::PointCloud2>
                               ("occupied_space",2);
        std::cout << "Starting ROS node for segmentation by Coyo-soft" << '\n';
        if (vizCnt) {
                marker_pub = nh.advertise<visualization_msgs::Marker>
                                     ("visualization_marker", 10);
                //initialize visualization_msgs, free space
                free_points.header.frame_id = baseFrame;
                free_points.header.stamp = ros::Time::now();
                free_points.ns = "visualization_points";
                free_points.action = visualization_msgs::Marker::ADD;
                free_points.pose.orientation.w =  1.0;
                free_points.id = 0;
                free_points.type = visualization_msgs::Marker::POINTS;
                free_points.scale.x = 0.1;
                free_points.scale.y = 0.1;
                free_points.scale.z = 0.1;
                free_points.color.g = 1.0f;
                free_points.color.a = 1.0;
                //initialize visualization_msgs, occupied space
                occ_points.header.frame_id = baseFrame;
                occ_points.header.stamp = ros::Time::now();
                occ_points.ns = "visualization_points";
                occ_points.action = visualization_msgs::Marker::ADD;
                occ_points.pose.orientation.w =  1.0;
                occ_points.id = 1;
                occ_points.type = visualization_msgs::Marker::POINTS;
                occ_points.scale.x = 0.1;
                occ_points.scale.y = 0.1;
                occ_points.scale.z = 0.1;
                occ_points.color.r = 1.0f;
                occ_points.color.a = 1.0;
        }
        if (vizSegCloud) {
                segClouds_pub = nh.advertise<sensor_msgs::PointCloud2>
                                        ("segmented_cloud",2);
        }
        //allocate color for segmentation
        colors = (int *)malloc(nClusters*sizeof(int));
        makeColors(colors,nClusters);
        return;
}

spaceSegmenter::~spaceSegmenter()
{

        free(colors);

}

float spaceSegmenter::makeFloat(unsigned char * byteArray)
{
        charToFloat S;
        for (size_t i = 0; i <4; i++)
        {
                S.byteStream[i]=byteArray[i];
        }
        return S.assembledFloat;
}




bool spaceSegmenter::cloudToBaseLink(
        const sensor_msgs::PointCloud2ConstPtr& in_cloud,
        sensor_msgs::PointCloud2 &out_cloud)
{
        //sensor_msgs::PointCloud2 out_cloud;
        bool sux = false;
        tf::StampedTransform transformTf;
        try{
                // ros::Time now = ros::Time::now();
                //tf_listener.waitForTransform("base_link", "head_rgbd_sensor_rgb_frame", ros::Time(0), ros::Duration(3.0));
                // tf_listener.lookupTransform("base_link", "head_rgbd_sensor_rgb_frame", ros::Time(0), transformTf);
                // pcl_ros::transformPointCloud("base_link",
                // transformTf,
                // *in_cloud,
                // out_cloud);

                sux = pcl_ros::transformPointCloud(baseFrame,
                                                   *in_cloud,
                                                   out_cloud,
                                                   tf_listener
                                                   );

                //printf("transformming\n");

        }
        catch (tf::TransformException ex) {
                //std::cout << "Erroring!!!!!" << '\n';
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
        }
        return sux;
}

void spaceSegmenter::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
        // pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        // pcl_conversions::toPCL(*msg, *cloud);
        // For this
        int n=msg->height*msg->width;
        std::cout << "Got a point cloud!  " << msg->height << "x" <<msg->width << " = "<< n <<'\n';
        std::cout << "Endianess: " << msg->is_bigendian <<'\n';
        std::cerr << "Point step" << msg->point_step<<'\n';
        std::cerr << "Row step" << msg->row_step<<'\n';

        std::cout << "Point Fields \n";
        //Check if pointcloud is compatible.
        //Currently only pointcloud2 with at least 3 xyz coordinates in float work
        if (msg->fields.size()<3) {
                std::cout << "This pointcloud does not have enough dimensions" << '\n';
                return;
        }
        for (int i = 0; i < 3; i++)
        {
                if ((short)msg->fields[i].datatype!=7)
                {
                        std::cout << "Field: "<<  msg->fields[i].name << " is not float can't process"<< '\n';
                        return;
                }
        }

        for (int i = 0; i < msg->fields.size(); i++) {
                std::cout << "Name " << msg->fields[i].name<<",";
                std::cout << "offset " << msg->fields[i].offset<<',';
                std::cout << "DataType " << (short)msg->fields[i].datatype<<',';
                std::cout << "Count " << msg->fields[i].count<<'\n';
        }
        //Convert to robot base frame
        sensor_msgs::PointCloud2 tfCloud;
        if(!cloudToBaseLink(msg,tfCloud))
        {
                ROS_ERROR("Something went wrong with tf\n");
                return;
        }

        //allocate memory for points in a simpler way
        point3 *space =
                (point3 *)malloc(n*sizeof(point3));
        int nValid = toPoint3(tfCloud,space);
        //Find max and min point for initialization
        point3 minP,maxP;
        getMinMax(space,maxP,minP,nValid);
        //allocate memory for codebook on host
        point3 *codebook =
                (point3 *)malloc(nClusters*sizeof(point3));
        //don't forget to allocate memory for partition
        int * partition =
                (int *)malloc(n*sizeof(int));
        //initializa codebook
        initializeCodebook(codebook,minP,maxP,nClusters);
        //Call quantizator
        kmeans(space,partition,codebook,iterations,
               nClusters,nValid);
        //printPoint3Array(codebook,nClusters);

        //Visualization
        if (vizCnt) {
                makeVizMsgAndPublish(codebook,nClusters);
        }
        if (vizSegCloud) {
                makeSegmentedCloudAndPublish(space,partition,nValid);
        }
        separateSpaceAndPublish(space,codebook,partition,nValid);
        //free host memory
        free(space); free(codebook); free(partition);
}

float spaceSegmenter::norm(point3 p)
{
        return sqrtf(p.x*p.x+p.y*p.y+p.z*p.z);
}

void spaceSegmenter::getMinMax(point3 * points,
                               point3 &max, point3 &min,
                               int nPoints)
{
        //max and min are the farthest and closest point to the frame origin
        float minDist = 10, maxDist = 0;
        for (size_t i = 0; i < nPoints; i++)
        {
                float tmp = norm(points[i]);
                if (tmp < minDist)
                {
                        min = points[i];
                        minDist = tmp;
                }
                if (tmp > maxDist)
                {
                        max = points[i];
                        maxDist =tmp;
                }
        }
}

int spaceSegmenter::toPoint3(sensor_msgs::PointCloud2 tfCloud,
                             point3 * points)
{
        /*tfcloud cloud on the robot base frame;
           points arrray of points to store valid points,
           returns: Number of no NaN poinst actually stored, continously on points

         */

        int point_step = tfCloud.point_step;
        int data_step = sizeof(float);
        std::vector<unsigned char> byteArray = tfCloud.data;
        int j=0;

        for (size_t i = 0; i < byteArray.size(); i+=point_step)
        {
                if(!std::isnan(makeFloat(&byteArray[i]))) {
                        points[j].x=makeFloat(&byteArray[i]);
                        points[j].y=makeFloat(&byteArray[i+data_step]);
                        points[j].z=makeFloat(&byteArray[i+2*data_step]);
                        j++;
                }

        }
        std::cout << "Got "<< j <<" non NaN points\n";
        return j;
}

void spaceSegmenter::makeVizMsgAndPublish(point3 *codebook, int nClusters)
{
        free_points.points.clear();
        occ_points.points.clear();
        for (int i = 0; i < nClusters; i++) {
                geometry_msgs::Point p;
                p.x = codebook[i].x;
                p.y = codebook[i].y;
                p.z = codebook[i].z;
                if (p.z < freeThr) {
                        free_points.points.push_back(p);

                }
                else{
                        occ_points.points.push_back(p);
                }
        }
        marker_pub.publish(free_points);
        marker_pub.publish(occ_points);
}

void spaceSegmenter::separateSpaceAndPublish(point3* space, point3* codebook,
                                             int * partition,
                                             int nPoints)
{
        // Ensemble pointcloud for free & occupied space.
        // and vomit them on to a fancy cloud2 msg.

        //check how many points are on free space
        //free space is assigned on centroids with z lower than freeThr
        int nFreePoints=0, nOccPoints =0;
        for (unsigned int i = 0; i < nPoints; i++)
        {
                if (codebook[partition[i]].z<=freeThr) {
                        nFreePoints++;
                }
        }
        nOccPoints =nPoints-nFreePoints;


        printf("%d points are on free space | %d points are on occupied space\n",
               nFreePoints, nOccPoints);
        //info for free space.
        sensor_msgs::PointCloud2 freeCloud;
        freeCloud.header.frame_id = baseFrame;
        freeCloud.header.stamp = ros::Time::now();
        freeCloud.width = nFreePoints;
        freeCloud.height = 1;
        freeCloud.is_bigendian = true;
        freeCloud.point_step = 4*4; //3 floats, x y z  rgb of 4 bytes
        freeCloud.row_step = nFreePoints*freeCloud.point_step;
        //info for occ space
        sensor_msgs::PointCloud2 occCloud;
        occCloud.header.frame_id = baseFrame;
        occCloud.header.stamp = ros::Time::now();
        occCloud.width = nOccPoints;
        occCloud.height = 1;
        occCloud.is_bigendian = true;
        occCloud.point_step = 4*4; //4 floats, x y z  rgb of 4 bytes
        occCloud.row_step = nOccPoints*occCloud.point_step;
        //Header for the cloud
        sensor_msgs::PointField pf;
        pf.name = "x";  pf.offset = 0;  pf.datatype = (unsigned char)7; pf.count =1;
        freeCloud.fields.push_back(pf);
        occCloud.fields.push_back(pf);
        pf.name = "y";  pf.offset = 4;  pf.datatype = (unsigned char)7; pf.count =1;
        freeCloud.fields.push_back(pf);
        occCloud.fields.push_back(pf);
        pf.name = "z";  pf.offset = 8;  pf.datatype = (unsigned char)7; pf.count =1;
        freeCloud.fields.push_back(pf);
        occCloud.fields.push_back(pf);
        pf.name = "rgb";  pf.offset = 12;  pf.datatype = (unsigned char)7; pf.count =1;
        freeCloud.fields.push_back(pf);
        occCloud.fields.push_back(pf);

        //Now ensemble the byteBlob
        //4 float of 4 bytes of nFreePoints 4*4*nFreePoints
        std::vector<unsigned char> byteBlobFree(freeCloud.row_step);
        std::vector<unsigned char> byteBlobOcc(occCloud.row_step);
        charToFloat changer;
        //This could be ewasily the most complex for in my
        //whoole life
        for (int i=0, j = 0, w=0; j< nPoints; j++)
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
                changer.assembledInt = colors[partition[j]];
                for (k = 0; k < 4; k++, u++) {
                        tmp[u]=changer.byteStream[k];
                }
                if (codebook[partition[j]].z<=freeThr)
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
        freeCloud_pub.publish(freeCloud);
        occCloud_pub.publish(occCloud);

}

void spaceSegmenter::makeSegmentedCloudAndPublish(point3 *space,
                                                  int *partition,
                                                  int nPoints)
{
        //Ensenbles a partitiones pointcloud, merely for
        // visualization purposes
        sensor_msgs::PointCloud2 segCloud;
        segCloud.header.frame_id = baseFrame;
        segCloud.header.stamp = ros::Time::now();
        segCloud.width = nPoints;
        segCloud.height = 1;
        segCloud.is_bigendian = true;
        segCloud.point_step = 4*4; //4 floats, x y z rgb of 4 bytes
        segCloud.row_step = nPoints*segCloud.point_step;
        std::vector<unsigned char> byteBlob(nPoints*segCloud.point_step);
        //Header for the cloud
        sensor_msgs::PointField pf;
        pf.name = "x";  pf.offset = 0;  pf.datatype = (unsigned char)7; pf.count =1;
        segCloud.fields.push_back(pf);
        pf.name = "y";  pf.offset = 4;  pf.datatype = (unsigned char)7; pf.count =1;
        segCloud.fields.push_back(pf);
        pf.name = "z";  pf.offset = 8;  pf.datatype = (unsigned char)7; pf.count =1;
        segCloud.fields.push_back(pf);
        pf.name = "rgb";  pf.offset = 12;  pf.datatype = (unsigned char)7; pf.count =1;
        segCloud.fields.push_back(pf);
        //Now ensemble the byteBlob
        charToFloat changer;
        for (int i = 0, j = 0;
             i < byteBlob.size();
             i+=segCloud.point_step, j++  )
        {
                changer.assembledFloat = space[j].x;
                int u=0,k;
                for (k = 0; k < 4; k++, u++) {
                        byteBlob[i+u]=changer.byteStream[k];
                }
                changer.assembledFloat = space[j].y;
                for (k = 0; k < 4; k++, u++) {
                        byteBlob[i+u]=changer.byteStream[k];
                }
                changer.assembledFloat = space[j].z;
                for (k = 0; k < 4; k++, u++) {
                        byteBlob[i+u]=changer.byteStream[k];
                }
                changer.assembledInt = colors[partition[j]];
                for (k = 0; k < 4; k++, u++) {
                        byteBlob[i+u]=changer.byteStream[k];
                }
        }
        segCloud.data = byteBlob;
        segClouds_pub.publish(segCloud);
}

void spaceSegmenter::makeColors(int *colors,int nClusters)
{
        //generates nClusters random rgba colors for later usage
        srand(0);
        int rgba;
        double v, vmin=0, vmax=nClusters;
        printf("Colors!\n" );
        for (int i = 0; i < nClusters; i++) {
                double r=1,g=1,b=1;
                //generate a random rgb tuple and pack on an int.
                // r=(unsigned char)(rand()%255);
                // g=(unsigned char)(rand()%255);
                // b=(unsigned char)(rand()%255);
                v=i;
                //printf("v is %f @ %d\n",v,i );
                if (v < vmin)
                        v = vmin;
                if (v > vmax)
                        v = vmax;
                double dv = vmax - vmin;

                if (v < (vmin + 0.25 * dv)) {
                        r = 0;
                        g = 4 * (v - vmin) / dv;
                } else if (v < (vmin + 0.5 * dv)) {
                        r = 0;
                        b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
                } else if (v < (vmin + 0.75 * dv)) {
                        r = 4 * (v - vmin - 0.5 * dv) / dv;
                        b = 0;
                } else {
                        g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
                        b = 0;
                }

                //from pcl docs,
                r*=255; g*=255; b*=255;
                printf("[%f,%f,%f]\n",r,g,b );

                rgba = ((int)r << 16 | (int)g << 8 | (int)b);
                //rgba = ((int)(r)) << 16 | ((int)(g)) << 8 | ((int)(b));
                colors[i] =rgba;
        }

        return;
}
