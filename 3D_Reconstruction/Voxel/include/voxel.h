#pragma once
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<float, 3, 1> Point;
const int VoxelSize_X = 1024;
const int VoxelSize_Y = 1024;
const int VoxelSize_Z = 512;
const float VoxelUnit = 0.01;

typedef struct _Voxel{
    float weight;
    float sdf;
    uchar red;
    uchar green;
    uchar blue;
}Voxel;

typedef struct _VoxelIndex{
    int index_x;
    int index_y;
    int index_z;
}VoxelIndex;


class VoxelGrid{
public:
    //init voxel data
    VoxelGrid();
    ~VoxelGrid();
    //크기떄문에 동적으로 할당하기
    Voxel ***voxel = new Voxel**[VoxelSize_X];
    Point centerVoxel(VoxelIndex voxelIndex);
};
