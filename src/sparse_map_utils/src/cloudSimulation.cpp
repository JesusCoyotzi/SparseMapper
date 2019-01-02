#include "sparse_map_utils/cloudSimulation.h"

const std::array<std::string,6> cloudSimulation::validFileType ={".obj",".ply",".pcd",".png",".pgm",".exr"};

cloudSimulation::cloudSimulation(ros::NodeHandle &nh) :
        cloud(new pcl::PointCloud<pcl::PointXYZ>)
{
        nh_=nh;
        ros::NodeHandle nh_priv("~");
        nh_priv.param<std::string>("pcd_folder",pcdFolder,".");
        //nh_priv.param<std::string>("pcd_file",pcdFile,"cloud.pcd");
        nh_priv.param<std::string>("csv_folder",csvFolder,pcdFolder);
        //nh_priv.param<std::string>("csv_file",csvFile,"results");
        nh_priv.param<std::string>("frame",frame,"map");
        nh_priv.param<int>("simulations_times",simTimes,1);
        nh_priv.param<float>("conversion_factor",conversionFactor,0.001);
        nh_priv.param<float>("voxel_size",voxSize,0.01);

        //Clusters to start
        //nh_priv.param<int>("clusters",clusters,1);
        //Max clusters to use
        nh_priv.param<int>("max_clusters",maxClusters,32);
        //minicul clusters per use
        nh_priv.param<int>("min_clusters",minClusters,16);
        nh_priv.param<int>("clusters_step",clustersStep,5);
        nh_priv.param<int>("iterations",iterations,6);
        nh_priv.param<std::string>("method",method,"kmeans ");

        cloudPub = nh_.advertise<sensor_msgs::PointCloud2>
                           ("out_cloud",1);
        codebookSub = nh_.subscribe
                              ("codebook",1,&cloudSimulation::codebookCallback,this);
        reconfigureClient = nh_.serviceClient<sparse_map_msgs::Reconfigure>
                                    ("segmentation_reconfigure");
        segmentationClient = nh_.serviceClient<sparse_map_msgs::QuantizeCloud>
                                     ("quantize_space");
        //cv::namedWindow( "cloudImg", cv::WINDOW_NORMAL );

        //Add trailins / to paths. If already present still works
        boost::filesystem::path p;
        p = pcdFolder;
        p += boost::filesystem::path::preferred_separator;
        pcdFolder = p.string();
        p = csvFolder;
        p += boost::filesystem::path::preferred_separator;
        csvFolder = p.string();
        return;
}

unsigned int cloudSimulation::getCloudFiles()
{
        std::cout << "Finding all cloud files on" << pcdFolder<<'\n';
        boost::filesystem::path p(pcdFolder);
        boost::filesystem::directory_iterator end_itr;

        // cycle through the directory
        for (boost::filesystem::directory_iterator itr(p); itr != end_itr; ++itr)
        {
                // If it's not a directory, list it. If you want to list directories too, just remove this check.
                if (boost::filesystem::is_regular_file(itr->path()))
                {
                        // assign current file name to current_file and echo it out to the console.
                        std::string fileExtension = itr->path().extension().string();
                        //  std::cout << fileExtension<< '\n';
                        for (size_t i = 0; i < 6; i++)
                        {
                                if(!fileExtension.compare(validFileType[i]))
                                {
                                        std::string currentFile(itr->path().string());
                                        std::cout << currentFile << std::endl;
                                        cloudFiles.push(currentFile);
                                }
                        }
                }
        }

        return cloudFiles.size();
}

bool cloudSimulation::loadNextCloud()
{
        if (cloudFiles.empty()) {
                return false;
        }

        pcdFile = cloudFiles.top();
        cloudFiles.pop();
        return loadCloud();
}

void cloudSimulation::monteCarlo()
{

        if(!reconfigureClient.waitForExistence(ros::Duration(5.0)))
        {
                std::cout << "Error reconfigure service not available" << '\n';
                return;
        }
        if (!segmentationClient.waitForExistence(ros::Duration(5.0)))
        {
                std::cout << "Error quantization service not available" << '\n';
                return;
        }
        simCounter=0;
        totalSimulations=0;


        clusters = minClusters;
        writeFileHeader();
        while (clusters<=maxClusters)
        {
                sparse_map_msgs::Reconfigure reconf;
                reconf.request.clusters.data = clusters;
                reconf.request.iterations.data = -1;

                if (reconfigureClient.call(reconf)) {
                        //writeFileHeader();
                        simCounter = 0;
                }
                else
                {
                        std::cout << "Error could not reconfigure" << '\n';

                        break;

                }
                while (simCounter<simTimes)
                {
                        sparse_map_msgs::QuantizeCloud qs;
                        sensor_msgs::PointCloud2 cloud_msg;
                        pcl::toROSMsg(*cloud,cloud_msg);
                        qs.request.cloud = cloud_msg;
                        startTime = ros::Time::now();
                        if (segmentationClient.call(qs))
                        {
                                ros::Duration execTime(ros::Time::now()-startTime);
                                std::cout << "Simulation: " << totalSimulations <<'\n';
                                std::cout << "Executed in: " << execTime << '\n';
                                unsigned long codes = qs.response.codebook.centroids.size();
                                std::cout << "Received a codebook of: " << codes<< '\n';
                                //printHist(qs.response.pa);
                                double distorsion = getDistorsion(qs.response.codebook, qs.response.partition);
                                //Means is always points / clusters
                                double histStdDev;
                                getHistStats(qs.response.histogram,histStdDev);
                                std::cout << "With overall distorsion of: " << distorsion<<'\n';
                                writeResult(totalSimulations,execTime.toSec(),
                                            distorsion,histStdDev,
                                            codes,clusters);

                                totalSimulations++;
                                simCounter++;
                                if(execTime.toSec()<1.0)
                                {
                                        //If the function is called twice during the same second rand will return same number
                                        ros::Duration(1.0).sleep();
                                }
                        }
                        else {
                                std::cout << "Cloud segment ommiting experiment" << '\n';
                                simCounter++;

                        }

                }
                clusters+=clustersStep;
        }
        return;
}

void cloudSimulation::startMonteCarlo()
{
        //Push the point cloud into the pipeline.
        //convert to msg and publish
        if(!reconfigureClient.waitForExistence(ros::Duration(5.0)))
        {
                std::cout << "Error reconfigure service not available" << '\n';
                return;
        }
        cloudSize = cloud->width*cloud->height;
        std::cout << "Starting simulation with " << cloudSize << " points\n";
        writeFileHeader();
        sendCloud();
        simCounter=0;
        totalSimulations=0;
        return;
}

bool cloudSimulation::loadCloud()
{
        // pcl::PointCloud<pcl::PointXYZ>::Ptr readCloud ( new pcl::PointCloud<pcl::PointXYZ>);
        //Load cloud into memory.
        std::cout << "Reading file " << pcdFile<<'\n';
        boost::filesystem::path p(pcdFile);
        std::string fileType(p.extension().string());
        bool succes = true;
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ> );
        if (!fileType.compare(".pcd")) {
                if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcdFile, *cloud) == -1)
                {
                        PCL_ERROR ("Couldn't read file  as PCD\n");
                        succes = false;
                }


        }
        else if(!fileType.compare(".ply")) {
                if(pcl::io::loadPLYFile<pcl::PointXYZ> (pcdFile, *cloud) == -1)
                {
                        PCL_ERROR("Could not read file as PLY");
                        succes = false;

                }
        }
        else if(!fileType.compare(".obj")) {
                if(pcl::io::loadOBJFile<pcl::PointXYZ> (pcdFile, *cloud) == -1)
                {
                        PCL_ERROR("Could not read file as OBJ");
                        succes = false;

                }
        }
        else if (!fileType.compare(".exr"))
        {
                //This is the file used by standford 3d2d dataset for XYZ data
                //It is on openEXR dataset
                //Reads it on a [x,y,z] format as a 32 bit float
                cv::Mat exrImg = cv::imread(pcdFile,cv::IMREAD_UNCHANGED);
                if (!exrImg.data) {
                        std::cout << "Error reading exr image" << '\n';
                        succes=  false;
                }
                // cv::imshow("cloudImg",exrImg);
                // cv::waitKey(15);
                std::cout << exrImg.type() << '\n';
                std::cout << exrImg.at<cv::Vec3f>() << '\n';
                succes =  makeCloudFromCloudImage(exrImg);
        }
        else if (!fileType.compare(".png") or !fileType.compare(".pgm"))
        {
                cv::Mat depthImg = cv::imread(pcdFile,cv::IMREAD_UNCHANGED);
                if (!depthImg.data) {
                        std::cout << "Error reading depth image" << '\n';
                        succes = false;
                }
                std::cout << depthImg.type() << '\n';
                std::cout << depthImg.at<unsigned short>() << '\n';
                succes = makeCloudFromDepthImage(depthImg);
        }
        else
        {
                std::cout << "Unkown/Unsupported file format" << '\n';
                succes = false;
        }
        if (voxSize) {
                subsampleCloud();
        }
        cloud->header.stamp = ros::Time::now().toNSec()/1000;
        cloud->header.frame_id = frame;
        cloudSize = cloud->points.size();
        std::cout << "Read cloud of " << cloudSize<< '\n';

        return succes;
}

void cloudSimulation::subsampleCloud()
{
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (voxSize, voxSize, voxSize);
        sor.filter (*cloud);
        return;
}

bool cloudSimulation::makeCloudFromDepthImage(cv::Mat & depthImg)
{
        boost::filesystem::path p(pcdFile);
        std::string stemName(p.stem().string());
        std::string parenName(p.parent_path().string());
        std::string calibFile(parenName+"/"+stemName+".json");

        // cv::FileStorage f(parenName+"/test.json", cv::FileStorage::WRITE);
        // cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
        // cv::Mat distCoeffs = (cv::Mat_<double>(5,1) << 0.1, 0.01, -0.001, 0, 0);
        // f << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
        // f.release();
        //
        std::cout << "Opening calibration file" << calibFile <<'\n';
        cv::FileStorage fs(calibFile,cv ::FileStorage::READ);
        if (!fs.isOpened()) {
                std::cout << "Could not open calibration file"<<"\n";
                return false;
        }

        float fx,fy,cx,cy;
        fs["camera_k_matrix"][0][0]>>fx;
        fs["camera_k_matrix"][1][1]>>fy;
        fs["camera_k_matrix"][0][2]>>cx;
        fs["camera_k_matrix"][1][2]>>cy;
        fs.release();

        //printf("[%f,%f,%f,%f]\n",fx,fy,cx,cy );

        int nRows = depthImg.rows;
        int nCols = depthImg.cols; // * depthImg.channels();
        unsigned short * dpth;
        for(int i = 0; i < nRows; ++i)
        {
                dpth = depthImg.ptr<unsigned short>(i);
                for (int j = 0; j < nCols; ++j)
                {
                        if (dpth[j]==0)
                        {
                                continue;
                        }
                        //std::cout << dpth[j] << '\n';
                        float Z =(float)dpth[j] * conversionFactor;
                        float X = (j-cx)*Z/fx;
                        float Y = (i-cy)*Z/fy;
                        //To change to a more convenient frmae:
                        pcl::PointXYZ punto(Z,X,Y);
                        cloud->points.push_back(punto);
                        //std::cout << punto << '\n';
                }
        }

        int pixels = cloud->points.size();
        int width = pixels;
        int height = 1;
        cloud->height = height;
        cloud->width = width;
        //std::cout << "Generating point cloud of: " << pixels <<" points \n";

        return true;
}

bool cloudSimulation::makeCloudFromCloudImage(cv::Mat & pcdImg)
{
        //cloud->points.resize(pixels);
        int nRows = pcdImg.rows;
        int nCols = pcdImg.cols; // * pcdImg.channels();
        cv::Vec3f* p;
        for(int i = 0; i < nRows; ++i)
        {
                p = pcdImg.ptr<cv::Vec3f>(i);
                for (int j = 0; j < nCols; ++j)
                {
                        if (p[j][0]!=0.0)
                        {
                                //file is rgb opencv is bgr
                                pcl::PointXYZ punto(p[j][2],p[j][1],p[j][0]);
                                cloud->points.push_back(punto);
                                //std::cout << punto << '\n';
                        }
                }
        }

        int pixels = cloud->points.size();
        int width = pixels;
        int height = 1;
        cloud->height = height;
        cloud->width = width;
        //std::cout << "Generating point cloud of: " << pixels <<" points \n";

        return true;

}

bool cloudSimulation::writeFileHeader()
{
        boost::filesystem::path p(pcdFile);
        std::string stemName(p.stem().string());
        std::string parenName(p.parent_path().string());
        fullResultsPath =csvFolder+stemName+"-"+method+".csv";
        // std::cout << parenName << '\n';
        // std::cout << stemName << '\n';
        std::cout << "Saving results at" << fullResultsPath <<'\n';
        std::ofstream resultsFile;
        resultsFile.open (fullResultsPath,std::ofstream::out);
        if(!resultsFile.is_open())
        {
                std::cout << "Error with file" << '\n';
                return false;
        }
        resultsFile << "# Meta-Data\n";
        resultsFile << "filename,method,iterations,points\n";
        resultsFile << pcdFile << ","<< method <<","
                    << iterations <<","<<cloudSize <<"\n";
        resultsFile << "# Data\n";
        resultsFile << "simulation,time,distorsion,mean,stddev,clusters,requested\n";
        resultsFile.close();
        return true;
}

bool cloudSimulation::writeResult(int sim,double secs,double distorsion,
                                   double histStdDev,
                                  unsigned long codesReceived, unsigned long requestedCodes
                                  )
{
        std::ofstream resultsFile;
        resultsFile.open(fullResultsPath,std::ofstream::app);
        if(!resultsFile.is_open())
        {
                std::cout << "Error with file" << '\n';
                return false;
        }
        resultsFile << std::fixed << std::setprecision(3);
        resultsFile << sim<<","<< secs<<","<< distorsion<<","
                    <<histStdDev<<","
                    << codesReceived<< "," <<requestedCodes <<"\n";
        resultsFile.close();
        return true;

}

bool cloudSimulation::writeResult(int sim,double secs,double distorsion,
                                  unsigned long codesReceived,
                                  unsigned long requestedCodes
                                  )
{
        std::ofstream resultsFile;
        resultsFile.open(fullResultsPath,std::ofstream::app);
        if(!resultsFile.is_open())
        {
                std::cout << "Error with file" << '\n';
                return false;
        }
        resultsFile << sim<<","<< secs<<","<< distorsion<<","
                    << codesReceived<< "," <<requestedCodes <<"\n";
        resultsFile.close();
        return true;

}


void cloudSimulation::codebookCallback(const sparse_map_msgs::codebook &msg)
{
        //Check the metrics: execution time, distorsion and missed codebooks
        ros::Duration execTime(ros::Time::now()-startTime);
        std::cout << "Simulation: " << totalSimulations <<'\n';
        std::cout << "Executed in: " << execTime << '\n';
        unsigned long codes =msg.centroids.size();
        std::cout << "Received a codebook of: " << codes<< '\n';
        double distorsion=getDistorsion(msg.centroids);
        std::cout << "With overall distorsion of: " << distorsion<<'\n';
        writeResult(totalSimulations,execTime.toSec(),distorsion,codes,clusters);
        if (simCounter<simTimes) {
                simCounter++;
                totalSimulations++;
                sendCloud();
        }
        else
        {
                if (clusters<=maxClusters) {
                        std::cout << "Reconfiguring" << '\n';
                        //Reconfigure parameters and try again with more clusters
                        //Also creates new file
                        sparse_map_msgs::Reconfigure reconf;
                        if (!method.compare("LBG")) {
                                //LBG can only operate on powers of 2
                                clusters*=2;
                        }
                        else
                        {
                                clusters+=clustersStep;
                        }
                        reconf.request.clusters.data = clusters;
                        reconf.request.iterations.data = -1;
                        if (reconfigureClient.call(reconf)) {
                                //writeFileHeader();
                                simCounter = 0;
                                totalSimulations++;
                                sendCloud();
                        }
                        else
                        {
                                std::cout << "Error could not reconfigure" << '\n';
                        }
                }
        }
        return;
}

void cloudSimulation::sendCloud()
{
        //send loaded cloud to pipeline
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud,cloud_msg);
        startTime = ros::Time::now();
        cloudPub.publish(cloud_msg);
        std::cout << "Cloud sent of: " << cloud->points.size()<<'\n';
        std::cout << cloud->points[0] << '\n';
        return;
}

void cloudSimulation::printHist(  std::vector<int> histogram)
{
        for (size_t i = 0; i < histogram.size(); i++) {
                printf("[%ld]:%d\n",i,histogram[i] );
        }
        return;
}

void cloudSimulation::getHistStats(std::vector<int> &hist,double &stdDev)
{
        double mean = 0.0;
        for (size_t i = 0; i < hist.size(); i++) {
                mean += hist[i];
        }
        mean /=hist.size();

        stdDev = 0.0;
        for (size_t i = 0; i < hist.size(); i++) {
                stdDev += (hist[i]-mean)*(hist[i]-mean);
        }
        stdDev /= hist.size();
        stdDev = sqrt(stdDev);
        return;
}

double cloudSimulation::getDistorsion(sparse_map_msgs::codebook cdbk,
                                      std::vector<int> histogram)
{
        double totalDistorsion=0;
        pointArray centroids = cdbk.centroids;
        for (unsigned int i = 0; i < cloud->points.size(); i++)
        {
                pcl::PointXYZ cloudPoint(cloud->points[i]);
                int pointLabel = histogram[i];
                double minDis = L2Norm(cloudPoint,centroids[pointLabel]);
                totalDistorsion+=minDis;
        }
        return totalDistorsion;
}

double cloudSimulation::getDistorsion(pointArray cdbk)
{
        double totalDistorsion=0;
        for (unsigned int i = 0; i < cloud->points.size(); i++)
        {
                pcl::PointXYZ cloudPoint(cloud->points[i]);
                double minDis = L2Norm(cloudPoint,cdbk[0]);
                for (unsigned int j = 1; j < cdbk.size(); j++)
                {
                        double dist= L2Norm(cloudPoint,cdbk[j]);
                        if (dist<minDis) {
                                minDis=dist;
                        }
                }
                totalDistorsion+=minDis;
                //std::cout << minDis << '\n';
        }
        return totalDistorsion;
}

double cloudSimulation::L1Norm(pcl::PointXYZ cloudPoint, geometry_msgs::Point cdbkPoint)
{
        //Calculates L1 Norm without haaving to conver types
        return (cloudPoint.x-cdbkPoint.x)*(cloudPoint.x-cdbkPoint.x)+(cloudPoint.y-cdbkPoint.y)*(cloudPoint.y-cdbkPoint.y)+(cloudPoint.z-cdbkPoint.z)*(cloudPoint.z-cdbkPoint.z);
}

double cloudSimulation::L2Norm(pcl::PointXYZ cloudPoint, geometry_msgs::Point cdbkPoint)
{
        //Calculates L1 Norm without haaving to conver types
        double L1= (cloudPoint.x-cdbkPoint.x)*(cloudPoint.x-cdbkPoint.x)+(cloudPoint.y-cdbkPoint.y)*(cloudPoint.y-cdbkPoint.y)+(cloudPoint.z-cdbkPoint.z)*(cloudPoint.z-cdbkPoint.z);
        return sqrt(L1);
}
