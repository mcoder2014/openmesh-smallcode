#ifndef LAPLACETRANSFORMATION_H
#define LAPLACETRANSFORMATION_H

#include <vector>
#include <map>

#include <Eigen/Dense>

#include "Mesh.h"

using namespace std;
using Eigen::Vector3d;
using Eigen::MatrixXf;

class LaplaceTransformation
{
public:

    // 设置原始模型
    inline void setSourceMesh(Mesh& mesh) {
        this->sourceMesh = mesh;
    }

    // 设置变化锚点
    void setMoveAnchors(vector<int>& moveAnchorIndex, vector<Vector3d>& moveAnchorPosition);
    void addMoveAnchor(int moveIndex, Vector3d position);

    void setFixedAnchors(vector<int>& fixedAnchorIndex);
    void addFixedAnchor(int fixedIndex);

    // 应用，返回变换后的 Mesh
    Mesh apply();

private:
    // 将要变形的网格模型
    Mesh sourceMesh;
    map<int, int> degreeMap;

    // 固定锚点
    vector<int> fixedAnchorIndex;
    // 变换锚点
    vector<int> moveAnchorIndex;
    vector<Vector3d> moveAnchorPosition;

    // 涉及到的矩阵运算
    MatrixXf matrixL;
    MatrixXf matrixLsT;
    MatrixXf matrixLsTLs;

    MatrixXf pointsX;
    MatrixXf pointsY;
    MatrixXf pointsZ;

    MatrixXf matrixLsTBX;
    MatrixXf matrixLsTBY;
    MatrixXf matrixLsTBZ;

    // 初始化矩阵
    void initMatrix();
    void buildMatrixLsTLs();
    void buildMatrixLsTB();

    // 解方程
    void calculate();

    // 应用变换坐标
    Mesh applyTransformation();

    // 计算每个顶点的度
    void getDegree();
};

#endif // LAPLACETRANSFORMATION_H
