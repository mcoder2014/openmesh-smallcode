#include <iostream>

#include <OpenMesh/Core/IO/MeshIO.hh>

#include "LaplaceDeformation.h"
#include "Mesh.h"

using namespace std;

int main()
{
    cout << "Test of Laplace Transformation" << endl;

    /// Load test Model
    Mesh sourceMesh;
    OpenMesh::IO::read_mesh(sourceMesh, "materials/cube.obj");

    // 构建拉普拉斯变形工具
    LaplaceDeformation laplaceDeformation;

    // 顶点 index   x     y     z
    //       0   -100  -100   100
    //       1   -100   100   100
    //       2   -100  -100  -100
    //       3   -100   100   100
    //
    //       4    100  -100   100
    //       5    100   100   100
    //       6    100  -100  -100
    //       7    100   100  -100

    laplaceDeformation.setSourceMesh(sourceMesh);

    // 添加固定锚点
    laplaceDeformation.addFixedAnchor(0);
    laplaceDeformation.addFixedAnchor(1);
    laplaceDeformation.addFixedAnchor(2);
    laplaceDeformation.addFixedAnchor(3);

    // 添加 移动锚点
    laplaceDeformation.addMoveAnchor(4, Vector3d(100, 200, 200));
    laplaceDeformation.addMoveAnchor(5, Vector3d(100, 200, -200));
    laplaceDeformation.addMoveAnchor(6, Vector3d(100, -200, 200));
    laplaceDeformation.addMoveAnchor(7, Vector3d(100, -200, -200));

    Mesh dstMesh = laplaceDeformation.apply();
    OpenMesh::IO::write_mesh(dstMesh, "dst.obj");

    return 0;
}
