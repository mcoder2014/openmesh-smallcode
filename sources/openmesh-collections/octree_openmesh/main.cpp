#include <iostream>
#include <fstream>
#include <string>
#include <map>

#include <Eigen/Dense>

#include "Mesh.h"
#include "BoundingBoxHelper.h"
#include "Octree.h"

using namespace std;

using Eigen::Vector3d;
using Eigen::AlignedBox3d;

// 向网格中添加面
void addFace(Mesh& mesh, int i, int j, map<pair<int, int>, int>& indexMap);

// 将网格的顶点保存下来，尝试使用功能三角化建模
void savePointsTxt(Mesh& mesh, string filePath);

/**
 * @brief remeshBottom
 * 用均匀扫描线，扫描模型底层，并进行 mesh 构建
 * @param mesh
 * @return
 */
Mesh remeshBottom(Octree& octree, size_t numX, size_t numY)
{
    Mesh dstMesh;
    AlignedBox3d boundingBox = octree.originBoundingBox;
    Vector3d extent = boundingBox.max() - boundingBox.min();
    double delta = std::max(extent.x(), std::max(extent.y(), extent.z()));
    delta /= std::min(numX, numY);

    // 记录某个位置扫描点加入 mesh 后的 index
    map<pair<int, int>, int> indexMap;

    // 从采样点位置发射射线
    Vector3d direction(0.0, 0.0, -1.0);
    Vector3d start = boundingBox.min();
    start.z() += 2 * extent.z();

    Vector3d point(start);
    for(size_t i = 0; i < numX; i++) {
        for(size_t j = 0; j < numY; j++) {

            Ray ray(point, direction, 0.000001);
            int faceIndex = -1;
            Vector3d intersectPoint = octree.closestIntersectionPoint(ray, &faceIndex);
            if(faceIndex > 0) {
                Mesh::VertexHandle vh = dstMesh.add_vertex(intersectPoint);
                indexMap[pair<int, int>(i, j)] = vh.idx();
            }
            point.y() += delta;
        }
        point.x() += delta;
        point.y() = start.y();
    }

    // 建立网格链接
    for(int i = 0; i < static_cast<int>(numX - 1); i++) {
        for(int j = 0; j < static_cast<int>(numY - 1); j++) {
            addFace(dstMesh, i, j, indexMap);
        }
    }

    return dstMesh;
}

/**
 * @brief getVertexHandle
 * 获得顶点 index，不存在返回 idx==-1
 * @param mesh
 * @param idx
 * @param indexMap
 * @return
 */
Mesh::VertexHandle getVertexHandle(Mesh& mesh, pair<int, int> idx, map<pair<int, int>, int>& indexMap)
{
    if(indexMap.find(idx) == indexMap.end()) {
        return Mesh::VertexHandle(-1);
    }
    return mesh.vertex_handle(static_cast<uint>(indexMap[idx]));
}

/**
 * @brief addFace
 * 向模型中加入一个面片
 * @param mesh
 * @param i
 * @param j
 * @param indexMap
 */
void addFace(Mesh& mesh, int i, int j, map<pair<int, int>, int>& indexMap)
{
    // 初始化为非法
    Mesh::VertexHandle a = getVertexHandle(mesh, pair<int, int>(i, j), indexMap);
    Mesh::VertexHandle b = getVertexHandle(mesh, pair<int, int>(i, j + 1), indexMap);
    Mesh::VertexHandle c = getVertexHandle(mesh, pair<int, int>(i + 1, j), indexMap);
    Mesh::VertexHandle d = getVertexHandle(mesh, pair<int, int>(i + 1, j + 1), indexMap);;

    vector<Mesh::VertexHandle> face;

    if(a.is_valid() && b.is_valid() && c.is_valid()) {
        face.push_back(d);
        face.push_back(b);
        face.push_back(a);
        mesh.add_face(face);
    }
    if(a.is_valid() && d.is_valid() && c.is_valid()) {
        face.clear();
        face.push_back(c);
        face.push_back(d);
        face.push_back(a);
        mesh.add_face(face);
    }
}

/**
 * @brief savePointsTxt
 * @param mesh
 * @param filePath
 */
void savePointsTxt(Mesh& mesh, string filePath)
{
    filePath.append(".txt");
    std::fstream outFile;
    outFile.open(filePath, ios::out | ios::trunc);

    Vector3d point;
    for(Mesh::VIter viter = mesh.vertices_begin(); viter != mesh.vertices_end(); viter++)
    {
        point = mesh.point(*viter);
        outFile << point.x() << " " << point.y() << " " << point.z() << "\n";
    }
    outFile.close();
}


int main(int argc, char *argv[])
{
    if(argc < 3) {
        cout << "mirror <sourceModel> <destinationModel>\n"
             << "support the file type as much as OpenMesh\n";
        return 0;
    }

    string sourceModelPath(argv[1]);
    string destModelPath(argv[2]);
    Mesh sourceModel;

    // 加载模型
    OpenMesh::IO::read_mesh(sourceModel, sourceModelPath);
    cout << "Load model finished..." << endl;

    Octree octree(sourceModel);
    cout << "Create octree finished..." << endl;

    Mesh destModel =remeshBottom(octree, 1000, 1000);
    cout << "Remesh bottom finished..." << endl;

    // 保存
    OpenMesh::IO::Options option;
    option = OpenMesh::IO::Options::Binary;
    OpenMesh::IO::write_mesh(destModel, destModelPath, option);
    savePointsTxt(destModel, destModelPath);

    return 0;
}
