#include "quantization.h"


void printVec(float vec[], int n)
{
        printf("[");
        printf("%f",vec[0]);
        for(int i= 1; i<n; i++)
        {
                printf(",%f",vec[i]);
        }
        printf("]\n");
        return;
}


void printVec(int vec[], int n)
{
        printf("[");
        printf("%d",vec[0]);
        for(int i= 1; i<n; i++)
        {
                printf(",%d",vec[i]);
        }
        printf("]\n");
        return;
}

void printPoint3(point3 p)
{
        printf("[%f,%f,%f]\n",p.x,p.y,p.z);
        return;
}

void printPoint3Array(point3 *p, int n)
{
        for(int i=0; i<n; i++)
        {
                printPoint3(p[i]);
        }
}

__device__ __host__ point3 addPoint3(point3 p1,point3 p2)
{
        point3 sum;
        sum.x=p1.x+p2.x;
        sum.y=p1.y+p2.y;
        sum.z=p1.z+p2.z;

        return sum;
}

__device__ __host__ point3 mulPoint3(point3 p, float s)
{
        point3 mul;
        mul.x=s*p.x;
        mul.y=s*p.y;
        mul.z=s*p.z;

        return mul;
}

