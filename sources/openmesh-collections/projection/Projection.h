#ifndef PROJECTION_H
#define PROJECTION_H

#include <vector>

#include <Eigen/Dense>

#include "Mesh.h"

using std::vector;
using Eigen::Vector3d;
using Eigen::Matrix4d;
using Eigen::Matrix3d;

class Projection
{
public:
    Projection(){}

    // 投影在 三点构成的面上
    Mesh project(Mesh& mesh, vector<Vector3d> face);

    // 齐次矩阵，投影用
    Matrix4d getProjectMatrix(vector<Vector3d>& face);

    // 获得坐标转换矩阵
    Matrix3d getTransMatrix(vector<Vector3d>& face);
};

#endif // PROJECTION_H
