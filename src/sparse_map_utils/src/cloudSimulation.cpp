#include "sparse_map_utils/cloudSimulation.h"

cloudSimulation::cloudSimulation(ros::NodeHandle &nh) :
        cloud(new pcl::PointCloud<pcl::PointXYZ>)
{
        nh_=nh;
        ros::NodeHandle nh_priv("~");
        nh_priv.param<std::string>("pcd_file",pcdFile,"cloud.pcd");
        nh_priv.param<std::string>("csv_file",csvFile,"results.csv");
        nh_priv.param<int>("simulations",simulations,1);
        nh_priv.param<int>("clusters",clusters,1);
        nh_priv.param<int>("iterations",iterations,1);
        nh_priv.param<std::string>("method",method,"kmeans ");

        cloudPub = nh_.advertise<sensor_msgs::PointCloud2>
                           ("out_cloud",1);
        codebookSub = nh_.subscribe
                              ("codebook",1,&cloudSimulation::codebookCallback,this);
        return;
}

bool cloudSimulation::loadCloud()
{
        // pcl::PointCloud<pcl::PointXYZ>::Ptr readCloud ( new pcl::PointCloud<pcl::PointXYZ>);
        //Load cloud into memory.
        std::cout << "Reading file " << pcdFile<<'\n';
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcdFile, *cloud) == -1)
        {
                PCL_ERROR ("Couldn't read file\n");
                return false;
        }

        return true;
}

bool cloudSimulation::writeFileHeader()
{
        std::ofstream resultsFile;
        resultsFile.open (csvFile,std::ofstream::out);
        if(!resultsFile.is_open())
        {
                std::cout << "Error with file" << '\n';
                return false;
        }
        resultsFile << pcdFile << ","<<method <<"," << clusters<<"," << iterations <<"\n";
        resultsFile << "simulation,time,distorsion,codebook\n";
        resultsFile.close();
        return true;
}

bool cloudSimulation::writeResult(int sim,double secs,double distorsion,unsigned int codes)
{
        std::ofstream resultsFile;
        resultsFile.open (csvFile,std::ofstream::app);
        if(!resultsFile.is_open())
        {
                std::cout << "Error with file" << '\n';
                return false;
        }
        resultsFile << sim<<","<< secs<<","<< distorsion<<","<< codes<<"\n";
        resultsFile.close();
        return true;

}

void cloudSimulation::startMonteCarlo()
{
        //Push the point cloud into the pipeline.
        //convert to msg and publish
        int cloudSize = cloud->width*cloud->height;
        std::cout << "Starting simulation with " << cloudSize<< " points";
        writeFileHeader();
        sendCloud();
        startTime = ros::Time::now();
}

void cloudSimulation::codebookCallback(const sparse_map_msgs::codebook &msg)
{
        ros::Duration execTime(ros::Time::now()-startTime);

        std::cout << "Simulation: " << simulations <<'\n';
        std::cout << "Executed in: " << execTime << '\n';
        std::cout << "Received a codebook of: " << msg.centroids.size()<< '\n';
        double distorsion=getDistorsion(msg.centroids);
        std::cout << "With overall distorsion of: " << distorsion<<'\n';
        if (simulations>0) {
                simulations--;
                sendCloud();
        }
        return;
}

void cloudSimulation::sendCloud()
{
        //send loaded cloud to pipeline
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud,cloud_msg);
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
