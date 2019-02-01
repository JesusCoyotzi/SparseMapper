#include "sparse_map_server/adjacencyMap.h"

voxelGrid::voxelGrid()
{
        this->stepX = 0;
        this->stepY = 0;
        this->stepZ = 0;
        this->cellsX = 0;
        this->cellsY = 0;
        this->cellsZ = 0;
}
voxelGrid::voxelGrid(float step)
{
        //basic constructor  use cubic voxels
        this->stepX = step;
        this->stepY = step;
        this->stepZ = step;
        isSet=true;
        return;
}

void voxelGrid::setStep(float step)
{
        this->stepX = step;
        this->stepY = step;
        this->stepZ = step;
        isSet = true;
        return;
}

void voxelGrid::setStep(float stepx,float stepy,float stepz)
{
        this->stepX = stepx;
        this->stepY = stepy;
        this->stepZ = stepz;
        isSet = true;
        return;
}

void voxelGrid::voxelize(std::vector<pointGeom> points)
{
        if (!isSet) {
                std::cout << "No step or cell number specified call set[Step|Cells]" << '\n';
                return;
        }
        getAOBB(points,minPoint,maxPoint );
        cellsX = ceil( (maxPoint.x-minPoint.x)/stepX );
        cellsY = ceil( (maxPoint.y-minPoint.y)/stepY );
        cellsZ = ceil( (maxPoint.z-minPoint.z)/stepZ );
        voxelGrd.resize(cellsX*cellsY*cellsZ);

        std::list<pointGeom> pointList(points.begin(),points.end());
        std::cout << "Voxelizing a volume of  " << voxelGrd.size() << "cells\n";
        printf("With [%d,%d,%d] cells\n",cellsX,cellsY,cellsZ);
        printf("Cell Size [%f,%f,%f]\n \n", stepX,stepY,stepZ);
        printf("With %ld points \n", pointList.size());
        // std::cout << "Min Point" << minPoint <<'\n';
        // std::cout << "Max Point" << maxPoint <<'\n';
        //copy to list since is faster to remove, push from it
        //Touchin size
        int idx = 0, i=0,j=0,k=0;
        pointGeom voxelI = minPoint;
        pointGeom voxelI_next;
        voxelI_next.x = minPoint.x + (1) * stepX;
        voxelI_next.y = minPoint.y + (1) * stepY;
        voxelI_next.z = minPoint.z + (1) * stepZ;

        int voxelized =0;
        while(!pointList.empty())
        {
                if (idx > voxelGrd.size())
                {
                        std::cout << "Over indexing!!! " << idx <<'\n';
                        break;
                }
                // printf("Writing on voxel[ %d], (%d,%d,%d)\n",
                //        idx, i,j,k);
                // printf("Next xovel (%f,%f,%f)\n",
                //        voxelI_next.x,voxelI_next.y,voxelI_next.z );
                std::list<pointGeom>::iterator it = pointList.begin();
                while (it != pointList.end())
                {
                        pointGeom q = *it;

                        bool isInside =
                                ( q.x>=voxelI.x) && ( q.x <= voxelI_next.x) &&
                                ( q.y>=voxelI.y ) && ( q.y <= voxelI_next.y ) &&
                                ( q.z>=voxelI.z ) && ( q.z <= voxelI_next.z );
                        if ( isInside)
                        {
                                it = pointList.erase(it);
                                voxelGrd[idx].push_back(q);
                                voxelized++;
                        }
                        else
                        {
                                ++it;
                        }

                }


                idx++;
                i++;
                if (i >= cellsX) {
                        i=0;
                        j++;
                }
                if (j >= cellsY) {
                        j=0;
                        k++;
                }
                if (k >= cellsZ) {
                        k=0;
                        std::cout << "Shoull not be here already overflowed" << '\n';
                        break;
                }
                //Overflows outside voxels why?
                //added >=

                voxelI.x = minPoint.x + i * stepX;
                voxelI.y = minPoint.y + j * stepY;
                voxelI.z = minPoint.z + k * stepZ;

                voxelI_next.x = minPoint.x + (i + 1) * stepX;
                voxelI_next.y = minPoint.y + (j + 1)* stepY;
                voxelI_next.z = minPoint.z + (k + 1) * stepZ;
        }
        isReady = true;
        std::cout << "Voxelixed :" <<  voxelized<< " points\n";
        return;
}

pointArray voxelGrid::getPointsInVoxel(pointGeom q, bool eigthN)
{
        //return all points in a 27 neighboor around point q
        if (!isReady) {
                std::cout << "Error: voxel grid not made, call voxelize" << '\n';
        }
        int i = (q.x -minPoint.x)/stepX;
        int j = (q.y -minPoint.y)/stepY;
        int k = (q.z -minPoint.z)/stepZ;
        pointArray closePoints;

        if (eigthN)
        {
                int maxI = i+1, minI = i-1;
                int maxJ = i+1, minJ = i-1;
                //Return all codes in a 8-neighboord around q voxel
                for (i=minJ; i < maxJ; i++)
                {
                        for (j = minI; i < maxI; i++)
                        {
                                int idxQ = i + j * cellsX + k * cellsY;
                                if (( idxQ > voxelGrd.size() ) && ( idxQ < 0 )) {
                                        voxel vx = voxelGrd[idxQ];
                                        closePoints.insert( closePoints.end(), vx.begin(), vx.end());
                                }
                        }
                }
        }
        else
        {
                //Return only local voxel, probably faster
                int idxQ = i + j * cellsX + k * cellsY;
                voxel vx = voxelGrd[idxQ];
                closePoints.insert( closePoints.end(), vx.begin(), vx.end());
        }


        return closePoints;
}

void voxelGrid::printVoxGrid()
{
        int n = 0,cnt=0;
        int i=0,j=0,k=0;
        pointGeom vxI = minPoint;
        pointGeom vxI_next;
        vxI_next.x = minPoint.x + (1) * stepX;
        vxI_next.y = minPoint.y + (1)* stepY;
        vxI_next.z = minPoint.z + (1) * stepZ;
        for (const voxel &vx : voxelGrd)
        {
                //BUG fucntion crashes if voxel step is not uniform in x,y,z
                printf("At voxel [%d]\n",n );
                printf("\t {%f,%f,%f} ||", vxI.x,vxI.y,vxI.z);
                printf("{%f,%f,%f} \n", vxI_next.x,vxI_next.y,vxI_next.z);
                for(const pointGeom &p : vx)
                {
                        printf("\t\t {%f,%f,%f}\n",p.x,p.y,p.z );
                        cnt=0;
                }
                n++;

                i++;
                if (i>cellsX) {
                        i=0;
                        j++;
                }
                if (j > cellsY) {
                        j=0;
                        k++;
                }
                if (k > cellsZ) {
                        std::cout << "Should not be here already overflowed" << '\n';
                }
                vxI.x = minPoint.x + i * stepX;
                vxI.y = minPoint.y + j * stepY;
                vxI.z = minPoint.z + k * stepZ;

                vxI_next.x = minPoint.x + (i + 1) * stepX;
                vxI_next.y = minPoint.y + (j + 1)* stepY;
                vxI_next.z = minPoint.z + (k + 1) * stepZ;

                cnt++;
        }
        std::cout << "Grid has : " << n<<'\n';
        return;
}

void voxelGrid::getAOBB(pointArray &points,
                        pointGeom &minCorner, pointGeom &maxCorner)
{
        float minDist = 10, maxDist = 0;
        minCorner = points[0];
        maxCorner = points[0];
        for (unsigned int i = 0; i < points.size(); i++)
        {
                float localX = points[i].x;
                float localY = points[i].y;
                float localZ = points[i].z;
                //Find max
                if (localX > maxCorner.x)
                {
                        maxCorner.x = localX;
                }
                if (localY > maxCorner.y)
                {
                        maxCorner.y = localY;
                }
                if (localZ > maxCorner.z)
                {
                        maxCorner.z = localZ;
                }
                //find minimun
                if (localX < minCorner.x)
                {
                        minCorner.x = localX;
                }
                if (localY < minCorner.y)
                {
                        minCorner.y = localY;
                }
                if (localZ < minCorner.z)
                {
                        minCorner.z = localZ;
                }

        }
        return;
}
