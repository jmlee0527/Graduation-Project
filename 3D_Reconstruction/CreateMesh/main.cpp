#include "../GeneratePointCloud/include/GeneratePointCloud.h"
#include "../Voxel/include/RayCast.h"
#include "include/createMesh.h"

int main(int argc, char* argv[])
{
    //첫번째 인자는 data 가 있는 파일 경로, 두 번째 인자로 받는 size 값은 0.02*size 씩 건너뛰어서 point 생성
    string filePath = argv[1];
    int startFile = atoi(argv[2]);
    int fileNum = atoi(argv[3]);
    int size = atoi(argv[4]);
    
    string rgbfile = "/rgb/";
    string depthfile = "/depth/";

    //GeneratePointCloud class 의 객체 선언
    GeneratePointCloud generatePointCloud;

    //rgb.txt, depth.txt, ground.txt 를 가져와서 비슷한 timestamp 대의 <rgb, depth, pose> tuple 을 만든다.
    map<double, vector<string> >  rgb_list = generatePointCloud.read_file_list(filePath + "/rgb.txt");
    map<double, vector<string> >  depth_list = generatePointCloud.read_file_list(filePath + "/depth.txt");
    map<double, vector<string> >  pose_list = generatePointCloud.read_file_list(filePath + "/groundtruth.txt");
    vector<pair<double, double> > rgb_depth_list = generatePointCloud.associate_two(rgb_list, depth_list);
    vector<pair<double, double> > depth_pose_list = generatePointCloud.associate_two(pose_list, depth_list);
    vector<tuple<double, double, double> > rgb_pose_depth_list = generatePointCloud.associate_three(rgb_depth_list, depth_pose_list);
    
    // groundtruth.txt 파일의 pose 를 모두 4x4 matrix 로 변경하여 poseMatrix 에 저장
    generatePointCloud.poseMatrix = generatePointCloud.read_trajectory(filePath + "/groundtruth.txt");
    //20 프레임만 사용한다.
    VoxelIndex minIndex;
    VoxelIndex maxIndex;
    
    RayCast rayCast;
    vector<VoxelIndex> findIndex;
    CreateMesh createMesh;
    minIndex.index_x = VoxelSize_X;
    minIndex.index_y = VoxelSize_Y;
    minIndex.index_z = VoxelSize_Z;
    maxIndex.index_x = 0;
    maxIndex.index_y = 0;
    maxIndex.index_z = 0;
    for (int i = startFile; i < fileNum; i += size)
    {
        string rgbPNG = filePath + rgbfile + to_string(get<0>(rgb_pose_depth_list[i])) + ".png";
        string depthPNG = filePath + depthfile + to_string(get<1>(rgb_pose_depth_list[i])) + ".png";
        Eigen::Matrix4f matrixPose = generatePointCloud.poseMatrix[get<2>(rgb_pose_depth_list[i])].cast<float>();
        generatePointCloud.generate_pointcloud(rgbPNG, depthPNG, matrixPose);
    
        Point PointCloud, Camera;
        Camera << matrixPose(0, 3), matrixPose(1, 3), matrixPose(2, 3);
        printf("Camera Origin : x[%f], y[%f],  z[%f]\n", Camera(0), Camera(1), Camera(2));

        //여기 부분을 thread 로 나누어서 작업을 해야한다.
        //point 의 개수를 나누어서 실행
        //lock 을 어디다 걸어야할까? sdf 를 update 하는 부분? findIndex 부터 lock?
        for (auto point : generatePointCloud.points) 
        {
            //m 단위를 cm 단위로 변경 -> m 단위로 통일
            PointCloud << point.x, point.y, point.z;
            findIndex = rayCast.rayCasting(Camera, PointCloud);
            for (const auto& voxelIndex : findIndex)
            {   
                float currentSDF = createMesh.voxelUpdate.getSDF(Camera, PointCloud, voxelIndex);
                createMesh.voxelUpdate.getColor(voxelIndex, point.red, point.green, point.blue);

                createMesh.voxelUpdate.updateSDF(voxelIndex, currentSDF);
                createMesh.voxelUpdate.updateWeight(voxelIndex);
                if (voxelIndex.index_x < minIndex.index_x)
                    minIndex.index_x = voxelIndex.index_x;
                if (voxelIndex.index_y < minIndex.index_y)
                    minIndex.index_y = voxelIndex.index_y;
                if (voxelIndex.index_z < minIndex.index_z)
                    minIndex.index_z = voxelIndex.index_z;

                if (voxelIndex.index_x > maxIndex.index_x)
                    maxIndex.index_x = voxelIndex.index_x;
                if (voxelIndex.index_y > maxIndex.index_y)
                    maxIndex.index_y = voxelIndex.index_y;
                if (voxelIndex.index_z > maxIndex.index_z)
                    maxIndex.index_z = voxelIndex.index_z;
            }
        }
    }
    if(minIndex.index_x < 0) minIndex.index_x = 0;
    if(minIndex.index_y < 0) minIndex.index_y = 0;
    if(minIndex.index_z < 0) minIndex.index_z = 0;
    if(maxIndex.index_x < 0) maxIndex.index_x = VoxelSize_X;
    if(maxIndex.index_y < 0) maxIndex.index_y = VoxelSize_Y;
    if(maxIndex.index_z < 0) maxIndex.index_z = VoxelSize_Z;
    printf("max x[%d] y[%d] z[%d], min x[%d] y[%d] z[%d]\n", maxIndex.index_x, maxIndex.index_y, maxIndex.index_z,
                                            minIndex.index_x, minIndex.index_y, minIndex.index_z);
    createMesh.generateMesh(VoxelSize_X, VoxelSize_Y, VoxelSize_Z, maxIndex, minIndex, 0.0);
    createMesh.writePLY(startFile);
    
    return 0;
}