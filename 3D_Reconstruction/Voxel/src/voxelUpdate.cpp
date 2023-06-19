#include "../include/voxelUpdate.h"
#include "cmath"

float VoxelUpdate::getSDF(const Point& camera, const Point& point, VoxelIndex targetIndex)
{
    Point voxelCenter;
    if(targetIndex.index_x <= 0 || targetIndex.index_y <= 0 || targetIndex.index_z <= 0) return 0;
    else
    {
        voxelCenter = centerVoxel(targetIndex);
        const Point voxelTocamera = voxelCenter - camera;
        const Point pointTocamera = point - camera;

        const float dist = pointTocamera.norm();
        const float projectDist = voxelTocamera.dot(pointTocamera)/dist;
        const float sdf = dist - projectDist;
        return sdf;
    }
}

void VoxelUpdate::updateSDF(VoxelIndex currentIndex, float currentSDF)
{
    if(currentIndex.index_x <= 0 || currentIndex.index_y <= 0 || currentIndex.index_z <= 0) return;
    else
    {
        if(voxel[currentIndex.index_x][currentIndex.index_y][currentIndex.index_z].sdf  == 1) 
            voxel[currentIndex.index_x][currentIndex.index_y][currentIndex.index_z].sdf = currentSDF;
        else
        {
            float prevSDF = voxel[currentIndex.index_x][currentIndex.index_y][currentIndex.index_z].sdf;
            float prevWeight = voxel[currentIndex.index_x][currentIndex.index_y][currentIndex.index_z].weight;
            //update current sdf
            voxel[currentIndex.index_x][currentIndex.index_y][currentIndex.index_z].sdf = 
                (prevWeight * prevSDF + weight*currentSDF)/(prevWeight+weight);
        }
    }
}

void VoxelUpdate::updateWeight(VoxelIndex currentIndex)
{
    if(currentIndex.index_x <= 0 || currentIndex.index_y <= 0 || currentIndex.index_z <= 0) return;
    else
    {
        float prevWeight = voxel[currentIndex.index_x][currentIndex.index_y][currentIndex.index_z].weight;
        //update voxel Weight
        voxel[currentIndex.index_x][currentIndex.index_y][currentIndex.index_z].weight =
            min(prevWeight + weight, weightMax);
    }
}

void VoxelUpdate::getColor(VoxelIndex centerIndex, uchar red, uchar green, uchar blue)
{
    if(centerIndex.index_x <= 0 || centerIndex.index_y <= 0 || centerIndex.index_z <= 0) return;
    else
    {
        voxel[centerIndex.index_x][centerIndex.index_y][centerIndex.index_z].red = red;
        voxel[centerIndex.index_x][centerIndex.index_y][centerIndex.index_z].blue = blue;
        voxel[centerIndex.index_x][centerIndex.index_y][centerIndex.index_z].green = green;
    }
}