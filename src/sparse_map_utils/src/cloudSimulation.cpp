#include "sparse_map_utils/cloudSimulation.h"

cloudSimulation::cloudSimulation(ros::NodeHandle &nh) :
        cloud(new pcl::PointCloud<pcl::PointXYZ>)
{
        nh_=nh;
        ros::NodeHandle nh_priv("~");
        nh_priv.param<std::string>("pcd_file",pcdFile,"cloud.pcd");
        nh_priv.param<std::string>("csv_file",csvFile,"results");
        nh_priv.param<std::string>("frame",frame,"map");
        nh_priv.param<int>("simulations_times",simTimes,1);

        //Clusters to start
        nh_priv.param<int>("clusters",clusters,1);
        //Max clusters to use
        nh_priv.param<int>("max_clusters",maxClusters,32);
        nh_priv.param<int>("clusters_step",clustersStep,5);
        nh_priv.param<int>("iterations",iterations,6);
        nh_priv.param<std::string>("method",method,"kmeans ");

        cloudPub = nh_.advertise<sensor_msgs::PointCloud2>
                           ("out_cloud",1);
        codebookSub = nh_.subscribe
                              ("codebook",1,&cloudSimulation::codebookCallback,this);
        reconfigureClient = nh_.serviceClient<sparse_map_msgs::Reconfigure>
                                    ("segmentation_reconfigure");

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

        if (!fileType.compare(".pcd")) {
                if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcdFile, *cloud) == -1)
                {
                        PCL_ERROR ("Couldn't read file  as PCD\n");
                        return false;
                }
        }
        else if(!fileType.compare(".ply")) {
                if(pcl::io::loadPLYFile<pcl::PointXYZ> (pcdFile, *cloud) == -1)
                {
                        PCL_ERROR("Could not read file as PLY");
                        return false;
                }
        }
        else if(!fileType.compare(".obj")) {
                if(pcl::io::loadOBJFile<pcl::PointXYZ> (pcdFile, *cloud) == -1)
                {
                        PCL_ERROR("Could not read file as OBJ");
                        return false;
                }
        }


        return true;
}

bool cloudSimulation::writeFileHeader()
{
        boost::filesystem::path p(csvFile);
        std::string stemName(p.stem().string());
        std::string parenName(p.parent_path().string());
        fullResultsPath = parenName+"/"+stemName + method
                          + std::to_string(cloudSize) +".csv";
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
        resultsFile << "simulation,time,distorsion,clusters\n";
        resultsFile.close();
        return true;
}

bool cloudSimulation::writeResult(int sim,double secs,double distorsion,
                                  unsigned long codesReceived, unsigned long requestedCodes)
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
                if (clusters<maxClusters) {
                        std::cout << "Reconfiguring" << '\n';
                        //Reconfigure parameters and try again with more clusters
                        //Also creates new file
                        sparse_map_msgs::Reconfigure reconf;
                        clusters+=clustersStep;
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
        cloud_msg.header.frame_id = frame;
        cloud_msg.header.stamp = ros::Time::now();
        startTime = ros::Time::now();
        cloudPub.publish(cloud_msg);
        return;
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
