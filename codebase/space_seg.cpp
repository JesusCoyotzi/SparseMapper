void initializeCodebook(point3* points,point3 * codebook, int nClusters, int n)
{
        //loop over all clusters
        printf("Initializing centroids\n");
        point3 centroid = getCentroid(points,n);

        float offset = 2.0/n;
        float inc = M_PI*(3.0-sqrtf(5.0));
        for (unsigned int i = 0; i < nClusters; i++)
        {
                float y = (i*offset-1)+offset/2;
                float r = sqrtf(1- pow(y,2));
                float phi = ((i+1)%n)*inc;
                float x = cos(phi)*r;
                float z = cos(phi)*r;
                codebook[i].x = x+centroid.x;
                codebook[i].y = y+centroid.y;
                codebook[i].z = z+centroid.z;
                //printPoint3(codebook[i]);
        }
        return;
}
