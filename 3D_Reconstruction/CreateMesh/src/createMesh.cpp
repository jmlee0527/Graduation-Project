#include "../include/createMesh.h"
#include <cmath>

void CreateMesh::getSDFArray(int x, int y, int z)
{ 
    // 주변 8개로 sdf 배열 얻기
    vertexSDF[0] = voxelUpdate.voxel[x][y][z].sdf;
    vertexSDF[1] = voxelUpdate.voxel[x+1][y][z].sdf;
    vertexSDF[2] = voxelUpdate.voxel[x+1][y][z+1].sdf;
    vertexSDF[3] = voxelUpdate.voxel[x][y][z+1].sdf;
    vertexSDF[4] = voxelUpdate.voxel[x][y+1][z].sdf;
    vertexSDF[5] = voxelUpdate.voxel[x+1][y+1][z].sdf;
    vertexSDF[6] = voxelUpdate.voxel[x+1][y+1][z+1].sdf;
    vertexSDF[7] = voxelUpdate.voxel[x][y+1][z+1].sdf;
}

int CreateMesh::checkSDFSign(int isolevel)
{
    int cubeIndex = 0;
    for(int i = 0; i < 8; i++)
    {
        if(vertexSDF[i] < isolevel)
        {
            int num = std::pow(2, i);
            cubeIndex |= num;
        }
    }
    return cubeIndex;
}

Point CreateMesh::getVertex(int edge_num, const Point& voxelCenter, float bound)
{
    Point vertex;
    if(edge_num == 0)
    {
        vertex <<   (voxelCenter(0) + voxelCenter(0) + VoxelUnit)/bound,
                    (voxelCenter(1) + voxelCenter(1))/bound,
                    (voxelCenter(2) + voxelCenter(2))/bound;
    }
    else if(edge_num == 1)
    {
        vertex <<   (voxelCenter(0) + VoxelUnit + voxelCenter(0) + VoxelUnit)/bound,
                    (voxelCenter(1) + voxelCenter(1))/bound,
                    (voxelCenter(2) + voxelCenter(2) + VoxelUnit)/bound;
    }
    else if(edge_num == 2)
    {   
        vertex <<   (voxelCenter(0) + voxelCenter(0) + VoxelUnit)/bound,
                    (voxelCenter(1) + voxelCenter(1))/bound,
                    (voxelCenter(2) + VoxelUnit + voxelCenter(2) + VoxelUnit)/bound;
    }
    else if(edge_num == 3)
    {
        vertex <<   (voxelCenter(0) + voxelCenter(0))/bound,
                    (voxelCenter(1) + voxelCenter(1))/bound,
                    (voxelCenter(2) + voxelCenter(2) + VoxelUnit)/bound;
    }
    else if(edge_num == 4)
    {
        vertex <<   (voxelCenter(0) + voxelCenter(0) + VoxelUnit)/bound,
                    (voxelCenter(1) + VoxelUnit + voxelCenter(1) + VoxelUnit)/bound,
                    (voxelCenter(2) + voxelCenter(2))/bound;
    }
    else if(edge_num == 5)
    {
        vertex <<   (voxelCenter(0) + VoxelUnit + voxelCenter(0) + VoxelUnit)/bound,
                    (voxelCenter(1) + VoxelUnit + voxelCenter(1) + VoxelUnit)/bound,
                    (voxelCenter(2) + voxelCenter(2) + VoxelUnit)/bound;
    }
    else if(edge_num == 6)
    {
        vertex <<   (voxelCenter(0) + VoxelUnit + voxelCenter(0))/bound,
                    (voxelCenter(1) + VoxelUnit + voxelCenter(1) + VoxelUnit)/bound,
                    (voxelCenter(2) + VoxelUnit + voxelCenter(2) + VoxelUnit)/bound;
    }
    else if(edge_num == 7)
    {
        vertex <<   (voxelCenter(0) + voxelCenter(0))/bound,
                    (voxelCenter(1) + VoxelUnit + voxelCenter(1) + VoxelUnit)/bound,
                    (voxelCenter(2) + voxelCenter(2) + VoxelUnit)/bound;
    }
    else if(edge_num == 8)
    {
        vertex <<   (voxelCenter(0) + voxelCenter(0))/bound,
                    (voxelCenter(1) + voxelCenter(1) + VoxelUnit)/bound,
                    (voxelCenter(2) + voxelCenter(2))/bound;
    }
    else if(edge_num == 9)
    {
        vertex <<   (voxelCenter(0) + VoxelUnit + voxelCenter(0)+ VoxelUnit)/bound,
                    (voxelCenter(1) + voxelCenter(1) + VoxelUnit)/bound,
                    (voxelCenter(2) + voxelCenter(2))/bound;
    }
    else if(edge_num == 10)
    {
        vertex <<   (voxelCenter(0) + VoxelUnit + voxelCenter(0)+ VoxelUnit)/bound,
                    (voxelCenter(1) + voxelCenter(1) + VoxelUnit)/bound,
                    (voxelCenter(2) + VoxelUnit + voxelCenter(2) + VoxelUnit)/bound;
    }
    else if(edge_num == 11)
    {
        vertex <<   (voxelCenter(0) + voxelCenter(0))/bound,
                    (voxelCenter(1) + voxelCenter(1) + VoxelUnit)/bound,
                    (voxelCenter(2) + VoxelUnit + voxelCenter(2) + VoxelUnit)/bound;
    }
    return vertex;
}

void CreateMesh::generateMesh(int voxelSizeX, int voxelSizeY, int voxelSizeZ, VoxelIndex maxIndex, VoxelIndex minIndex, float isolevel)
{
    //vertex point
    // voxel 그리드 순회
    for (int x = minIndex.index_x; x < maxIndex.index_x; x++) 
    {
        for (int y = minIndex.index_y; y < maxIndex.index_y; y++) 
        {
            for (int z = minIndex.index_z; z < maxIndex.index_z; z++) 
            {
                VoxelIndex voxelIndex;
                voxelIndex.index_x = x;
                voxelIndex.index_y = y;
                voxelIndex.index_z = z;
                getSDFArray(x, y, z);
                int cubeIndex = 0;
                cubeIndex = checkSDFSign(isolevel);
                if(cubeIndex < 255 && cubeIndex > 0)
                {
                    // 삼각형 생성
                    Triangle triangle;
                    MeshInfo mesh;
                    Point vertex;
                    Point voxelCenter;
                    for (int i = 0; kTriangleTable[cubeIndex][i] != -1; i += 3) 
                    { 
                        int vertexIndex1 = kTriangleTable[cubeIndex][i];
                        int vertexIndex2 = kTriangleTable[cubeIndex][i+1];
                        int vertexIndex3 = kTriangleTable[cubeIndex][i+2];

                        triangle.e0 = vertexIndex1;
                        triangle.e1 = vertexIndex2;
                        triangle.e2 = vertexIndex3;
                        triangleVertex.push_back(triangle);
                        voxelCenter = voxelUpdate.centerVoxel(voxelIndex);
                        for(int k = 0; k < 3; k ++)
                        {
                            if(k == 0)
                            {
                                vertex = getVertex(vertexIndex1, voxelCenter, 2.0);
                                mesh.x = vertex(0);
                                mesh.y = vertex(1);
                                mesh.z = vertex(2);
                                mesh.red = voxelUpdate.voxel[x][y][z].red;
                                mesh.green = voxelUpdate.voxel[x][y][z].green;
                                mesh.blue = voxelUpdate.voxel[x][y][z].blue;
                                PointVector.push_back(mesh);
                            }
                            else if(k == 1)
                            {   
                                vertex = getVertex(vertexIndex2, voxelCenter, 2.0);
                                mesh.x = vertex(0);
                                mesh.y = vertex(1);
                                mesh.z = vertex(2);
                                mesh.red = voxelUpdate.voxel[x][y][z].red;
                                mesh.green = voxelUpdate.voxel[x][y][z].green;
                                mesh.blue = voxelUpdate.voxel[x][y][z].blue;
                                PointVector.push_back(mesh);
                            }
                            else if(k == 2)
                            {
                                vertex = getVertex(vertexIndex3, voxelCenter, 2.0);
                                mesh.x = vertex(0);
                                mesh.y = vertex(1);
                                mesh.z = vertex(2);
                                mesh.red = voxelUpdate.voxel[x][y][z].red;
                                mesh.green = voxelUpdate.voxel[x][y][z].green;
                                mesh.blue = voxelUpdate.voxel[x][y][z].blue;
                                PointVector.push_back(mesh);
                            }
                        }
                    }
                }else continue;
            }
        }
    }
}

void CreateMesh::writePLY(int num) 
{
    printf("start writting\n");
    std::string filename = "mesh" + to_string(num) + ".ply";
    std::ofstream file(filename);
    if (!file) 
    {
        std::cerr << "Failed to create PLY file: " << filename << std::endl;
        return;
    }

    // PLY 파일 헤더 작성
    file << "ply" << std::endl;
    file << "format ascii 1.0" << std::endl;
    file << "element vertex " << PointVector.size() << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "property uchar red" << std::endl;
    file << "property uchar green" << std::endl;
    file << "property uchar blue" << std::endl;
    file << "element face " << triangleVertex.size() << std::endl;
    file << "property list uint8 int32 vertex_indices" << std::endl;
    file << "end_header" << std::endl;
    for(const MeshInfo& mesh : PointVector) 
    {
        file << mesh.x << " " << mesh.y << " " << mesh.z << " " << to_string(mesh.red)
            << " " << to_string(mesh.green) << " " << to_string(mesh.blue) << std::endl;
    }
    int i = 0;
    for(const Triangle& triangle : triangleVertex)
    {
        file << "3 " << i << " " << i+1 << " " << i + 2 << std::endl;
        i += 3;
    }

    file.close();
    std::cout << "PLY file created: " << filename << std::endl;
}