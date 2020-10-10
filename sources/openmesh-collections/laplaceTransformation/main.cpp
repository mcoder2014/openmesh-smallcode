#include <iostream>

#include <OpenMesh/Core/IO/MeshIO.hh>

#include "LaplaceTransformation.h"
#include "Mesh.h"

using namespace std;

int main()
{
    cout << "Test of Laplace Transformation" << endl;

    /// Load test Model
    Mesh sourceMesh;
    OpenMesh::IO::read_mesh(sourceMesh, "materials/cube.obj");

    // 构建拉普拉斯变形工具
    LaplaceTransformation laplaceTransformation;

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

    laplaceTransformation.setSourceMesh(sourceMesh);

    // 添加固定锚点
    laplaceTransformation.addFixedAnchor(0);
    laplaceTransformation.addFixedAnchor(1);
    laplaceTransformation.addFixedAnchor(2);
    laplaceTransformation.addFixedAnchor(3);

    // 添加 移动锚点
    laplaceTransformation.addMoveAnchor(4, Vector3d(100, 200, 200));
    laplaceTransformation.addMoveAnchor(5, Vector3d(100, 200, -200));
    laplaceTransformation.addMoveAnchor(6, Vector3d(100, -200, 200));
    laplaceTransformation.addMoveAnchor(7, Vector3d(100, -200, -200));

    Mesh dstMesh = laplaceTransformation.apply();
    OpenMesh::IO::write_mesh(dstMesh, "dst.obj");

    return 0;
}
