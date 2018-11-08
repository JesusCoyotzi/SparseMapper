void adjacencyGraph::saveAdjGraph(std::string filename, pointArray centroids, float ** adjG)
{
        //Saves the adjacency matrix to a file to disk.
        std::cout << "Writing to "<<filename<<"\n";
        std::ofstream graphOut;
        graphOut.open(filename.c_str());
        graphOut<<"Clusters: " <<edges  << "\n";
        graphOut<<"Codebook: x,y,z,label\n";
        for(int i=0; i<edges; i++)
        {
                graphOut<<centroids[i].x<<",";
                graphOut<<centroids[i].y<<",";
                graphOut<<centroids[i].z<<",";
                graphOut<<i<<"\n";
        }
        graphOut<<"Adjacency Matrix:\n";
        for(int i=0; i<edges; i++)
        {
                for (int j = 0; j < edges; j++) {
                        graphOut<<adjG[i][j]<<",";

                }
                graphOut<<"\n";
        }
        graphOut.close();
        return;
}
