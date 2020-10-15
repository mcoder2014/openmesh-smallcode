#include <iostream>
#include <string>

#include <Eigen/Dense>

#include "Mesh.h"
#include "BoundingBoxHelper.h"
#include "Octree.h"

using namespace std;

using Eigen::Vector3d;
using Eigen::AlignedBox3d;

/**
 * @brief createGrid
 * 创建一个 XoY 平面的网格
 * @return
 */
Mesh createGrid(Vector3d center, size_t numX, size_t numY, double delta)
{
    Mesh grid;
    Vector3d start = center;
    start.x() -= (numX * delta / 2);
    start.y() -= (numY * delta / 2);

    // 添加顶点
    Vector3d point(start);
    for(size_t i = 0; i < numX; i++) {
        for(size_t j = 0; j < numY; j++) {
            grid.add_vertex(point);
            point.y() += delta;
        }
        point.x() += delta;
        point.y() = start.y();
    }

    // 添加边
    vector<Mesh::VertexHandle> face;
    for(size_t i = 0; i < numX-1; i++) {
        for(size_t j = 0; j < numY-1; j++) {
            Mesh::VertexHandle a = grid.vertex_handle(i * numY + j);
            Mesh::VertexHandle b = grid.vertex_handle(i * numY + j + 1);
            Mesh::VertexHandle c = grid.vertex_handle((i+1) * numY + j);
            Mesh::VertexHandle d = grid.vertex_handle((i+1) * numY + j + 1);

            face.clear();
            face.push_back(a);
            face.push_back(b);
            face.push_back(d);
            grid.add_face(face);

            face.clear();
            face.push_back(a);
            face.push_back(d);
            face.push_back(c);
            grid.add_face(face);
        }
    }
    return grid;
}

Mesh createGridFromBoundingBox(AlignedBox3d boundingBox)
{
    Vector3d center = boundingBox.center();
    Vector3d extent = boundingBox.max() - boundingBox.min();
    center.z() -= extent.z();
    size_t numX = 1000;
    size_t numY = 1000;
    double delta = std::max(extent.x(), std::max(extent.y(), extent.z()));
    delta /= std::min(numX, numY);

    return createGrid(center, numX, numY, delta);
}

void gridDeform(Mesh& grid, Octree& octree)
{
    Vector3d direction(0.0, 0.0, 1.0);

    for(Mesh::VIter viter = grid.vertices_begin(); viter != grid.vertices_end(); viter++)
    {
        Ray ray(grid.point(*viter), direction, 0.000001);
        int faceIndex = -1;
        Vector3d intersectPoint = octree.closestIntersectionPoint(ray, &faceIndex);
        if(faceIndex > 0) {
            grid.point(*viter) = intersectPoint;
        }
    }
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
    cout << "Load model success..." << endl;

    Octree octree(sourceModel);
    cout << "create octree success..." << endl;

    Eigen::AlignedBox3d boundingBox = octree.originBoundingBox;
    Mesh destModel = createGridFromBoundingBox(boundingBox);
    gridDeform(destModel, octree);

    // 保存
    OpenMesh::IO::Options option;
    option = OpenMesh::IO::Options::Binary;
    OpenMesh::IO::write_mesh(destModel, destModelPath, option);

    return 0;
}
