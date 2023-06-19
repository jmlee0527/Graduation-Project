cmake_minimum_required(VERSION 3.0)
project(CreateMesh)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
add_executable(mesh ../../GeneratePointCloud/src/GeneratePointCloud.cpp ../../Voxel/src/voxel.cpp ../../Voxel/src/voxelUpdate.cpp
                ../../Voxel/src/RayCast.cpp ../src/CreateMesh.cpp main.cpp)
target_link_libraries(mesh ${OpenCV_LIBS})
target_link_libraries(mesh Eigen3::Eigen)


