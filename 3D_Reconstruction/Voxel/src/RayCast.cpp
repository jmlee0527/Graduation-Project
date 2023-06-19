#include "../include/RayCast.h"
#include <cmath>
#include <cfloat>

VoxelIndex RayCast::offsetIndex(VoxelIndex inputIndex)
{
    VoxelIndex targetIndex;
    targetIndex.index_x = inputIndex.index_x + VoxelSize_X/2;
    targetIndex.index_y = inputIndex.index_y + VoxelSize_Y/2;
    targetIndex.index_z = inputIndex.index_z;
    return targetIndex;
}
VoxelIndex RayCast::pointToVoxel(const Point& point)
{
    VoxelIndex index;
    index.index_x = std::floor(point(0) / VoxelUnit);
    index.index_y = std::floor(point(1) / VoxelUnit);
    index.index_z = std::floor(point(2) / VoxelUnit);
    return index;
}

// 뒤에 어느정도 더 해주기 voxel 의 4배 정도
bool RayCast::endRay(VoxelIndex currentIndex, VoxelIndex targetIndex, 
    int truncatedX, int truncatedY, int truncatedZ)
{
    if((currentIndex.index_x != targetIndex.index_x + truncatedX * 5) && 
        (currentIndex.index_y != targetIndex.index_y + truncatedY * 5) &&
        (currentIndex.index_z != targetIndex.index_z + truncatedZ * 5)) return true;
    else return false;
}

vector<VoxelIndex> RayCast::rayCasting(const Point& camera, const Point& point)
{
    vector<VoxelIndex> findVoxelIndex;
    VoxelIndex currentIndex;
    VoxelIndex targetIndex;
    VoxelIndex diff;
    VoxelIndex inputIndex;
    
    //Ray 방향
    Point rayDirection;
    rayDirection = (point - camera);

    Point step;
    step << ((rayDirection(0) < 0) ? -1 : 1),
            ((rayDirection(1) < 0) ? -1 : 1),
            ((rayDirection(2) < 0) ? -1 : 1);

    currentIndex = pointToVoxel(camera);
    targetIndex = pointToVoxel(point);

    int truncatedX = (targetIndex.index_x - currentIndex.index_x < 0) ? -1 : 1;
    int truncatedY = (targetIndex.index_y - currentIndex.index_y < 0) ? -1 : 1;
    int truncatedZ = (targetIndex.index_z - currentIndex.index_z < 0) ? -1 : 1;

    double next_voxel_boundary_x = (currentIndex.index_x + step(0)) * VoxelUnit; // correct
    double next_voxel_boundary_y = (currentIndex.index_y + step(1)) * VoxelUnit; // correct
    double next_voxel_boundary_z = (currentIndex.index_z + step(2)) * VoxelUnit; // correct

    double tMaxX = (rayDirection(0) != 0) ? (next_voxel_boundary_x - camera(0))/rayDirection(0) : DBL_MAX; 
    double tMaxY = (rayDirection(1) != 0) ? (next_voxel_boundary_y - camera(1))/rayDirection(1) : DBL_MAX; 
    double tMaxZ = (rayDirection(2) != 0) ? (next_voxel_boundary_z - camera(2))/rayDirection(2) : DBL_MAX; 

    Point delta;
    delta << ((rayDirection(0)!=0) ? VoxelUnit/rayDirection(0) * step(0) : DBL_MAX),
             ((rayDirection(1)!=0) ? VoxelUnit/rayDirection(1) * step(1) : DBL_MAX),
             ((rayDirection(2)!=0) ? VoxelUnit/rayDirection(2) * step(2) : DBL_MAX);

    diff.index_x = 0;
    diff.index_y = 0;
    diff.index_z = 0;
        
    bool neg_ray=false;
    if (currentIndex.index_x != targetIndex.index_x && rayDirection(0) < 0) { diff.index_x--; neg_ray=true; }
    if (currentIndex.index_y != targetIndex.index_y && rayDirection(1) < 0) { diff.index_y--; neg_ray=true; }
    if (currentIndex.index_z != targetIndex.index_z && rayDirection(2) < 0) { diff.index_z--; neg_ray=true; }

    inputIndex = offsetIndex(currentIndex);
    if(inputIndex.index_x < VoxelSize_X && inputIndex.index_y < VoxelSize_Y && inputIndex.index_z < VoxelSize_Z) 
        findVoxelIndex.push_back(inputIndex);
    if (neg_ray) 
    {
        currentIndex.index_x += diff.index_x;
        currentIndex.index_y += diff.index_y;
        currentIndex.index_z += diff.index_z;
        
        inputIndex = offsetIndex(currentIndex);
        if(inputIndex.index_x < VoxelSize_X && inputIndex.index_y < VoxelSize_Y && inputIndex.index_z < VoxelSize_Z) 
            findVoxelIndex.push_back(inputIndex);
    }

    while(endRay(currentIndex, targetIndex, truncatedX, truncatedY, truncatedZ))
    {
        if (tMaxX < tMaxY) {
            if (tMaxX < tMaxZ) 
            {
                currentIndex.index_x += step(0);
                tMaxX += delta(0);
            } 
            else 
            {
                currentIndex.index_z += step(2);
                tMaxZ += delta(2);
            }
        } 
        else {
            if (tMaxY < tMaxZ) 
            {
                currentIndex.index_y += step(1);
                tMaxY += delta(1);
            } 
            else 
            {
                currentIndex.index_z += step(2);
                tMaxZ += delta(2);
            }
        }
        
        inputIndex = offsetIndex(currentIndex);
        if(inputIndex.index_x < VoxelSize_X && inputIndex.index_y < VoxelSize_Y && inputIndex.index_z < VoxelSize_Z) 
            findVoxelIndex.push_back(inputIndex);
    }
    return findVoxelIndex;
}