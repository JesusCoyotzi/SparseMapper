#include "map_serializer.h"

map_serializer::map_serializer(std::string filename)
{
        printf("Starting map serializer by Coyo\n");
        map_sub = nh_.subscribe("/map", 1,
                                &map_serializer::serialize, this);
        this->file = filename;
        return;

}

void map_serializer::serialize(const nav_msgs::OccupancyGrid &map)
{
        std::ofstream outfile(file.c_str(),std::ofstream::out);
        int width=map.info.width;
        int height=map.info.height;
        float resolution = map.info.resolution;
        int orig_x=-floor(map.info.origin.position.x/resolution);
        int orig_y=-floor(map.info.origin.position.y/resolution);


        printf("Got map: %dx%d cells @ %f [m/cell] \n", width,height,resolution );
        printf("Origin @ (%d,%d) \n",orig_x,orig_y);

        outfile << "Resolution " << resolution<<std::endl;
        outfile << "Size " << width <<" "<<height<<" "<<std::endl;
        outfile << "Origin " << orig_x<<" "<<orig_y  <<std::endl;
        for (size_t x = 0; x < width; x++)
        {
                for (size_t y= 0; y< height; y++)
                {
                        int value = map.data[x+ width * y];
                        if(value==0)
                        {
                                printf("Free space @ (%d,%d)\n",width,height );
                        }
                        else if(value>0)
                        {
                                printf("Blocked space @ (%d,%d)\n",width,height );
                                value=1;
                        }
                        outfile <<x <<" "<<y<<" "<<value<<std::endl;
                }
        }
        outfile.close();
        printf("Finished serialization\n" );
        return;
}
