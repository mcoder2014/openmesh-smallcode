#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <assert.h>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <Eigen/Dense>

#include <Mesh.h>

#include "DelaunayTriangulation.h"
#include "Vector3D.h"

using namespace std;

/**
 * @brief loadNodes
 * 读取随机顶点
 * @param pnodes
 * @param fname
 * @param type
 * @return
 */
bool loadNodes(std::vector<Vector3D*>& pnodes, std::string fname)
{
    fstream fin(fname);
    if(!fin){
        cout << "File Open Error\n";
        return false;
    }

    string tmp;
    while (getline(fin, tmp)) {
        std::istringstream ssIn(tmp);
        std::string tmpx[3];
        ssIn >> tmpx[0] >> tmpx[1] >> tmpx[2];

        pnodes.push_back(
                    new Vector3D(stod(tmpx[0]), stod(tmpx[1]), stod(tmpx[2])));

    }
    fin.close();
    return true;
}

/**
 * @brief getMesh
 * 构建 openmesh 模型
 * @param pnodes
 * @param pelements
 * @return
 */
Mesh getMesh(std::vector<Vector3D*> pnodes, vector<tuple<int, int, int>*> faces)
{
    Mesh mesh;

    // 插入顶点
    Mesh::Point point;
    for(Vector3D* node : pnodes)
    {
        point.x() = node->X;
        point.y() = node->Y;
        point.z() = node->Z;

        mesh.add_vertex(point);
    }

    // 插入面片
    vector<Mesh::VertexHandle> face;
    for (tuple<int,int,int>* element : faces) {
        face.clear();
        face.push_back(mesh.vertex_handle(get<0>(*element)));
        face.push_back(mesh.vertex_handle(get<1>(*element)));
        face.push_back(mesh.vertex_handle(get<2>(*element)));

        mesh.add_face(face);
    }

    return mesh;
}

int main(int argc, char *argv[])
{

    if(argc < 3) {
        cout << "delaunay_triangle <sourcePoints> <destinationModel>\n"
             << "support the file type as much as OpenMesh\n";
        return 0;
    }

    string sourcePointsPath(argv[1]);
    string destModelPath(argv[2]);

    // Load nodes
    std::vector<Vector3D*> nodes;
    loadNodes(nodes, sourcePointsPath);

    // Triangulation
    DelaunayTriangulation triangulation;
    vector<tuple<int, int, int>*> faces = triangulation.getTriangulationResult(nodes);
    Mesh dstMesh = getMesh(nodes, faces);

    // 保存
    OpenMesh::IO::Options option;
    option = OpenMesh::IO::Options::Binary;
    OpenMesh::IO::write_mesh(dstMesh, destModelPath, option);

    return 0;
}
