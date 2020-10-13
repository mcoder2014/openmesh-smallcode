#include <iostream>

#include <OpenMesh/Core/IO/MeshIO.hh>

#include "Mesh.h"
#include "Projection.h"

using namespace std;

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

    // 投影
    Projection projection;
    Mesh destModel = projection.ProjectZ(sourceModel);

    // 保存
    OpenMesh::IO::Options option;
    option = OpenMesh::IO::Options::Binary;
    OpenMesh::IO::write_mesh(destModel, destModelPath, option);

    return 0;
}
