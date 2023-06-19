#pragma once
#include <iostream>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "voxel.h"

using namespace std;

const float weight = 1.0;
const float weightMax = 100000.0; // 만에서 10만

class VoxelUpdate : public VoxelGrid {
public:
    VoxelUpdate() : VoxelGrid() {};
    ~VoxelUpdate() {}
    //update sdf, weight, color
    float getSDF(const Point& camera, const Point& point, VoxelIndex targetIndex);
    void getColor(VoxelIndex centerIndex, uchar red, uchar green, uchar blue);
    void updateSDF(VoxelIndex currentIndex, float currentSDF);
    void updateWeight(VoxelIndex currentIndex);
};