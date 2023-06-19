#include "../include/GeneratePointCloud.h"

// 초기 카메라 parameter setting
// ROS default 값을 이용
GeneratePointCloud::GeneratePointCloud()
{
    offset = 0.0;
    max_difference = 0.02;
    //default ROS
    scalingFactor = 5000;
    centerX = 319.5;
    centerY = 239.5;
    focalLength_x = 525;
    focalLength_y = 525;
}

//txt 파일을 읽어와서 timestamp 를 double type로 받아서 저장해서 map<double, vector<string> > lst 를 반환하는 함수
map<double, vector<string> > GeneratePointCloud::read_file_list(string filename)
{
    map<double, vector<string> > lst;
    ifstream file(filename.c_str());
    if (!file) {
        cerr << "Cannot open " << filename << endl;
        exit(1);
    }
    string line;
    while (getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        stringstream ss(line);
        vector<string> parts;
        string part;
        while (ss >> part) parts.push_back(part);
    
        if (parts.size() < 2) continue;

        double timestamp;
        stringstream(parts[0]) >> timestamp;
        lst[timestamp] = vector<string>(parts.begin() + 1, parts.end());
    }
    return lst;
}

// read_file_list(string filename) 에서 나온 결과 list 두 개의 timestamp 를 확인하여, 오차가 0.02 안의 것들은 
// 하나의 pair 로 반드는 함수
// ex) rgb.txt 와 depth.txt 파일의 timestamp 차이가 0.02 미만일 경우 둘을 하나의 pair 로 만든다.
vector<pair<double, double> > GeneratePointCloud::associate_two(map<double, vector<string> > first_list,
                                             map<double, vector<string> > second_list)
{
    vector<double> first_keys, second_keys;
    for (auto it = first_list.begin(); it != first_list.end(); it++) first_keys.push_back(it->first);
    for (auto it = second_list.begin(); it != second_list.end(); it++) second_keys.push_back(it->first);

    sort(first_keys.begin(), first_keys.end());
    sort(second_keys.begin(), second_keys.end());
    vector<pair<double, double> > matches;
    for (double a : first_keys) {
        vector<double>::iterator lower_it = lower_bound(second_keys.begin(), second_keys.end(), a - max_difference - offset);
        vector<double>::iterator upper_it = upper_bound(second_keys.begin(), second_keys.end(), a + max_difference - offset);
        for (vector<double>::iterator b_it = lower_it; b_it != upper_it; b_it++) {
            double b = *b_it;
            double time_diff = abs(a - (b + offset));
            if (time_diff < max_difference) {
                matches.push_back(make_pair(a, b));
                second_keys.erase(b_it);
                break;
            }
        }
    }
    return matches;
}

// associate_two 에서 만든 pair 를 depth 를 기준으로 tuple type 로 만들어서 저장
// <rgb, depth> 와 <depth, ground.txt> 에서 같은 값의 depth 만 추출하여 <rgb, depth, groundtruth> 의 tuple로 만든다.
vector<tuple<double, double, double> > GeneratePointCloud::associate_three(vector<pair<double, double> > rgb_depth, 
                                                        vector<pair<double, double> > pose_depth)
{
    vector<tuple<double, double, double>> rgb_pose_depth;
    for (const auto& rgbdepth : rgb_depth) 
    {
        for(const auto& posedepth : pose_depth)
        {
            if(rgbdepth.second == posedepth.second) rgb_pose_depth.push_back(make_tuple(rgbdepth.first, rgbdepth.second,posedepth.first));
        }
    }

    return rgb_pose_depth;
}

//transform44, read_trajectory 설명
//read_trajectory에서 groundtruth.txt의 timestamp 와 x,y,z, qx,qy,qw,qz 를 읽는다.
//transform44 함수에 pose와 quternion 값을 인자로 넣어서 4x4 행렬을 만든다.
//q.toRotationMatrix()는 쿼터니언 q를 회전 행렬(rotation matrix)로 변환한다. 
//쿼터니언 q를 회전 행렬로 변환하여, T 행렬의 첫 번째 열부터 세 번째 열까지를 회전 행렬로 설정하여 T를 반환한다.
Matrix4d GeneratePointCloud::transform44(const vector<double>& v)
{
    Matrix4d T = Matrix4d::Identity();
    T.block(0, 3, 3, 1) = Eigen::Vector3d(v[1], v[2], v[3]);
    Eigen::Quaterniond q(v[7], v[4], v[5], v[6]);
    T.block(0, 0, 3, 3) = q.toRotationMatrix();
    return T;
}

PoseMap GeneratePointCloud::read_trajectory(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: could not open file '" << filename << "'" << std::endl;
        exit(1);
    }

    PoseMap traj;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::istringstream iss(line);
        std::vector<double> values((std::istream_iterator<double>(iss)), std::istream_iterator<double>());

        if (values.size() != 8) {
            std::cerr << "Warning: line has " << values.size() << " values, expected 8" << std::endl;
            continue;
        }

        if (values[4] == 0 && values[5] == 0 && values[6] == 0 && values[7] == 0) {
            std::cerr << "Warning: quaternion values are all zero" << std::endl;
            continue;
        }

        if (std::any_of(values.begin(), values.end(), [](double v) { return std::isnan(v); })) {
            std::cerr << "Warning: line contains NaN values" << std::endl;
            continue;
        }

        traj[values[0]] = transform44(values);
    }

    file.close();

    return traj;
}

//비슷한 timestamp 때의 rgb.png 와 depth.png 와 4x4 matrix를 인자로 받아서 
// double Z = static_cast<double>(depth_value) / scalingFactor;
// if (Z == 0) continue;
// double X = (u - centerX) * Z / focalLength_x;
// double Y = (v - centerY) * Z / focalLength_y;
// Vector4f vec_org(X, Y, Z, 1);
// Vector4f vec_transf = transforms * vec_org;
// 식을 통해서 point 를 생성하고 맴버변수인 points vextor 에 push_back한다.
void GeneratePointCloud::generate_pointcloud(const string& rgb_file, const string& depth_file, const Eigen::Matrix4f& transforms)
{
    cv::Mat rgb = cv::imread(rgb_file);
    cv::Mat depth = cv::imread(depth_file, cv::IMREAD_UNCHANGED);

    PointsData add_points;
    for (int v = 0; v < rgb.rows; v += 1) {
        for (int u = 0; u < rgb.cols; u += 1) {
            cv::Vec3b color = rgb.at<cv::Vec3b>(v, u);
            ushort depth_value = depth.at<ushort>(v, u);
            double Z = static_cast<double>(depth_value) / scalingFactor;
            if (Z == 0) continue;
            double X = (u - centerX) * Z / focalLength_x;
            double Y = (v - centerY) * Z / focalLength_y;
            Eigen::Vector4f vec_org(X, Y, Z, 1);
            Eigen::Vector4f vec_transf = transforms * vec_org;

            //openCV = b g r - >rgb 순서로 넣기
            add_points={vec_transf[0], vec_transf[1], vec_transf[2], color[2], color[1], color[0]};
            points.push_back(add_points);
        }
    }
}

//push_back 한 맴버 변수 points vector를 인자로 받아 이를 .ply 파일로 변경해준다.
void GeneratePointCloud::write_ply(string ply_file, vector<PointsData> points)
{
    std::ofstream file(ply_file);
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex " << points.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "property uchar red\n";
    file << "property uchar green\n";
    file << "property uchar blue\n";
    file << "end_header\n";
    for (auto point : points) {
        file << to_string(point.x) + " " + to_string(point.y) + " " + to_string(point.z) 
                + " " + to_string(point.red) + " " + to_string(point.green) + " " + to_string(point.blue) << "\n";
    }
    file.close();
}