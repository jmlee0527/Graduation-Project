#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <cmath>
#include <string>
#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <tuple>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef map<double, Matrix4d> PoseMap;
struct PointsData{
    float x;
    float y;
    float z;
    uchar red;
    uchar green;
    uchar blue;
};

class GeneratePointCloud{
public :
    GeneratePointCloud();
    ~GeneratePointCloud() {}
    vector<PointsData> points; // ply 파일에 넣을 point 구조체 vector
    PoseMap poseMatrix; // grountruth.txt 파일의 pose를 4x4 matrix 로 저장하는 변수
    map<double, vector<string> > read_file_list(string filename);
    vector<pair<double, double> > associate_two(map<double, vector<string> > first_list, map<double, vector<string> > second_list);
    vector<tuple<double, double, double> > associate_three(vector<pair<double, double> > rgb_depth, 
                                                        vector<pair<double, double> > pose_depth);
    PoseMap read_trajectory(const string& filename);
    Matrix4d transform44(const vector<double>& v);

    //make point cloud and make ply file
    void generate_pointcloud(const string& rgb_file, const string& depth_file, const Eigen::Matrix4f& transforms);
    void write_ply(string ply_file, vector<PointsData> points);

private:
    double offset;
    float max_difference;
    int scalingFactor;
    float centerX;
    float centerY;
    int focalLength_x;
    int focalLength_y;
};