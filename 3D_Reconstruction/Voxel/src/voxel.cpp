#include "../include/voxel.h"

VoxelGrid::VoxelGrid()
{
    for(int i = 0; i < VoxelSize_X; i++) 
    {
        voxel[i] = new Voxel*[VoxelSize_Y];
        for(int j = 0; j < VoxelSize_Y; j++) voxel[i][j] = new Voxel[VoxelSize_Z];
    }

    for(int i = 0; i < VoxelSize_X; i++)
    {
        for(int j = 0; j < VoxelSize_Y; j++)
        {
            for(int k = 0; k < VoxelSize_Z; k++)
            {
                voxel[i][j][k].weight = 0;
                voxel[i][j][k].sdf = 0.0;
                voxel[i][j][k].red = 0;
                voxel[i][j][k].green = 0;
                voxel[i][j][k].blue = 0;
            }
        }
    }
}

VoxelGrid::~VoxelGrid()
{
    for(int i = 0; i < VoxelSize_X; i++) 
    {
        for(int j = 0; j < VoxelSize_Y; j++) delete[] voxel[i][j];
        delete[] voxel[i];
    }
    delete[] voxel;
}

Point VoxelGrid::centerVoxel(VoxelIndex voxelIndex)
{
    Point voxelCenter;
    voxelCenter << static_cast<float>((voxelIndex.index_x - VoxelSize_X/2)*VoxelUnit + VoxelUnit/2), 
                    static_cast<float>((voxelIndex.index_y - VoxelSize_Y/2)*VoxelUnit + VoxelUnit/2), 
                    static_cast<float>((voxelIndex.index_z)*VoxelUnit + VoxelUnit/2);
    return voxelCenter;
}