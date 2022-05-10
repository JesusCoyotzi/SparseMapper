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

