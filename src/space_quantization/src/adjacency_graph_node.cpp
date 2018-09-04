#include "adjacency_graph.h"
int main(int argc, char *argv[])
{
        ros::init(argc,argv,"graph_node");
        ros::NodeHandle nh;
        adjacencyGraph ag(nh);

        ros::spin();
        return 0;
}
