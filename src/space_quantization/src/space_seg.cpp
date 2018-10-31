#include "space_seg.h"

spaceSegmenter::spaceSegmenter(ros::NodeHandle nh)
{
        ros::NodeHandle nh_priv("~");
        nh_priv.param<int>("nClusters",nClusters,8);
        nh_priv.param<int>("iterations",iterations,16);
        nh_priv.param<bool>
                ("publishSegmentation",pubSegSpace,true);
        sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>
                            ("cloud",10,
                            &spaceSegmenter::cloudCallback,
                            this);
        labeledCloudPub = nh.advertise<sensor_msgs::PointCloud2>
                                  ("labeled_cloud",2);
        // freeCloudPub = nh.advertise<sensor_msgs::PointCloud2>
        //                            ("free_space",2);
        // occCloudPub = nh.advertise<sensor_msgs::PointCloud2>
        //                            ("occ_space",2);
        quantizedSpace_pub = nh.advertise<space_quantization::quantizedSpace>
                                     ("quantized_space",2);
        codebook_pub = nh.advertise<space_quantization::codebook>
                                     ("codebook",2);

        std::cout << "Starting ROS node for segmentation by Coyo-soft" << '\n';
        return;
}

spaceSegmenter::~spaceSegmenter()
{
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

void spaceSegmenter::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
        // pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        // pcl_conversions::toPCL(*msg, *cloud);
        // For this
        cloudFrame = msg->header.frame_id;
        stamp = msg->header.stamp;
        int n=msg->height*msg->width;
        std::cout << "Got a point cloud! of  " << msg->height << "x" <<msg->width << " = "<< n <<'\n';
        std::cout << "Endianess: " << msg->is_bigendian <<'\n';
        std::cout << "Point step: " << msg->point_step<<'\n';
        std::cout << "Row step: " << msg->row_step<<'\n';
        std::cout << "Frame id: " << cloudFrame << '\n';
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

        //allocate memory for points in a simpler way
        point3 *space =
                (point3 *)malloc(n*sizeof(point3));
        int nValid = toPoint3(*msg,space);

        //allocate memory for codebook on host
        point3 *codebook =
                (point3 *)malloc(nClusters*sizeof(point3));
        //don't forget to allocate memory for partition
        int * partition =
                (int *)malloc(n*sizeof(int));
        //And the histogram
        int * histogram =
                (int*)malloc(nClusters*sizeof(int));
        //initializa codebook
        //Find max and min point for initialization
        point3 minP,maxP;
        getMinMax(space,maxP,minP,nValid);
        initializeCodebook(codebook,minP,maxP,nClusters);
        // initializeCodebook(space,codebook,nClusters,nValid);
        //Call quantizator
        kmeans(space,partition,codebook,histogram,iterations,
               nClusters,nValid);

        labelSpaceAndPublish(space,codebook,partition,histogram,nValid);
        //free host memory
        free(space); free(codebook); free(partition); free(histogram);
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

void spaceSegmenter::makeCodebookMsg(std::vector<geometry_msgs::Point> &msg,
                                     point3 *codebook, int* histogram,
                                     int nClusters)
{
        //Ensemnles a geometry_msgs::Point[] vector to send the codebook to other topics
        //Basically finishes takin the quantization library types to pure ros msg
        for (int i = 0; i < nClusters; i++)
        {
                if (histogram[i]>0) {
                        geometry_msgs::Point chg;
                        chg.x=codebook[i].x;
                        chg.y=codebook[i].y;
                        chg.z=codebook[i].z;
                        msg.push_back(chg);
                        // msg[i].x=codebook[i].x;
                        // msg[i].y=codebook[i].y;
                        // msg[i].z=codebook[i].z;
                }
        }
}

void spaceSegmenter::makeCloudHeader(sensor_msgs::PointCloud2 &cloud, int points)
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

        //Header for the cloud
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
spaceSegmenter::separateSpaceAndPublish(point3* space,
                                        point3* codebook,
                                        int * partition,
                                        int * histogram,
                                        int nPoints)
{
        // Ensemble pointcloud for free & occupied space.
        // and vomit them on to a fancy cloud2 msg.
        //Count free points and occupied points
        int nFreePoints=0, nOccPoints =0;
        for (unsigned int i = 0; i < nPoints; i++)
        {
                if (codebook[partition[i]].z<=freeThr) {
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
        std::vector<unsigned char> byteBlobFree(nFreePoints);
        std::vector<unsigned char> byteBlobOcc(nOccPoints);
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
                changer.assembledInt = partition[j];
                for (k = 0; k < 4; k++, u++) {
                        tmp[u]=changer.byteStream[k];
                }
                // for (unsigned int v = 0; v < 4*4; v++) {
                //
                //         byteBlobLabel[i+v]=tmp[v];
                // }
                // i+=labeledCloud.point_step;
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

        std::vector<geometry_msgs::Point> centroids(nClusters);
        makeCodebookMsg(centroids,codebook, histogram,nClusters);
        //make quantizedSpace obj
        space_quantization::quantizedSpace qs;
        qs.space = occCloud;
        qs.codebook = centroids;
        //Allows for publish only segmented cloud
        // if (pubSegSpace)
        // {
        //         freeCloudPub.publish(freeCloud);
        //         occCloudPub.publish(occCloud);
        // }
        quantizedSpace_pub.publish(qs);
        return;
}

void
spaceSegmenter::labelSpaceAndPublish(point3* space,
                                     point3* codebook,
                                     int * partition,
                                     int * histogram,
                                     int nPoints)
{
        // Ensemble pointcloud for free & occupied space.
        // and vomit them on to a fancy cloud2 msg.

        // set info for Cloud.
        sensor_msgs::PointCloud2 labeledCloud;
        makeCloudHeader(labeledCloud,nPoints);

        //Now ensemble the byteBlob
        //4 float of 4 bytes of nFreePoints 4*4*nFreePoints
        std::vector<unsigned char> byteBlobLabel(labeledCloud.row_step);
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
                changer.assembledInt = partition[j];
                for (k = 0; k < 4; k++, u++) {
                        tmp[u]=changer.byteStream[k];
                }
                for (unsigned int v = 0; v < 4*4; v++) {

                        byteBlobLabel[i+v]=tmp[v];
                }
                i+=labeledCloud.point_step;

        }
        labeledCloud.data = byteBlobLabel;

        std::vector<geometry_msgs::Point> centroids;
        makeCodebookMsg(centroids,codebook,histogram,nClusters);
        //make quantizedSpace obj
        space_quantization::quantizedSpace qs;
        qs.space = labeledCloud;
        qs.codebook = centroids;
        //Allows for publish only segmented cloud
        if (pubSegSpace)
        {
                labeledCloudPub.publish(labeledCloud);
        }
        quantizedSpace_pub.publish(qs);

        space_quantization::codebook cdbk;
        cdbk.centroids = centroids;
        cdbk.header.frame_id =  cloudFrame;
        cdbk.header.stamp = stamp;
        codebook_pub.publish(cdbk);
        return;
}
