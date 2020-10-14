#include "Projection.h"

#include <iostream>

/**
 * @brief Projection::project
 * @param mesh
 * @param face
 *
 * 将模型投影到面上
 *
 * @return
 */
Mesh Projection::project(Mesh &mesh, vector<Eigen::Vector3d> face)
{
    assert(mesh.n_vertices() >0);
    Mesh dst;
    dst.assign(mesh);

    Matrix4d projectMat = getProjectMatrix(face);

    // 计算顶点矩阵
    Eigen::MatrixXd points = Eigen::MatrixXd::Ones(4, static_cast<int>(dst.n_vertices()));
    Mesh::Point point;
    for(Mesh::VIter viter = dst.vertices_begin(); viter != dst.vertices_end(); viter++)
    {
        point = dst.point(*viter);
        int index = (*viter).idx();
        points(0, index) = point.x();
        points(1, index) = point.y();
        points(2, index) = point.z();
    }

    // 计算投影
    points = projectMat * points;

    // 保存投影结果
    for(int i = 0; i <static_cast<int>(dst.n_vertices()); i++)
    {
        point.x() = points(0, i);
        point.y() = points(1, i);
        point.z() = points(2, i);
        dst.point(dst.vertex_handle(static_cast<uint>(i))) = point;
    }
    return dst;
}

/**
 * @brief Projection::getProjectMatrix
 * @param face
 *
 * 投影矩阵
 * M 是空间转换矩阵
 *
 * P' = inv(M)* [1 0 0 0  * M * P
 *              0 1 0 0
 *              0 0 0 0
 *              0 0 0 1]
 * @return
 */
Eigen::Matrix4d Projection::getProjectMatrix(vector<Eigen::Vector3d> &face)
{
    Matrix4d baseEigen = Matrix4d::Identity();
    baseEigen(2, 2) = 0;

    Eigen::Matrix3d transMat = getTransMatrix(face);
    Eigen::Matrix3d invTransMat = transMat.inverse();

    std::cout << "transMat:\n" << transMat << std::endl;
    std::cout << "invTransMat:\n" << invTransMat << std::endl;

    /// 获得齐次矩阵
    Eigen::Matrix4d homoTransMat;
    Eigen::Matrix4d homoInvTransMat;
    homoTransMat.topLeftCorner(3,3) = transMat;
    homoInvTransMat.topLeftCorner(3,3) = invTransMat;

    homoTransMat.col(3) << 0, 0, 0, 1;
    homoTransMat.row(3) << 0, 0, 0, 1;
    homoInvTransMat.col(3) << 0, 0, 0, 1;
    homoInvTransMat.row(3) << 0, 0, 0, 1;

    std::cout << "homeTransMat:\n" << homoTransMat << std::endl;
    std::cout << "homeInvTransMat:\n" << homoInvTransMat << std::endl;

    // 返回投影矩阵
    return homoInvTransMat * baseEigen * homoTransMat;
}

/**
 * @brief Projection::getTransMatrix
 * 获得坐标系转换矩阵 M 3x3
 * @param face
 * @return
 */
Eigen::Matrix3d Projection::getTransMatrix(vector<Eigen::Vector3d> &face)
{
    assert(face.size() >= 3);

    // 三个顶点
    Vector3d& a = face[0];
    Vector3d& b = face[1];
    Vector3d& c = face[2];

    // 两个向量
    Vector3d ba = a - b;
    Vector3d bc = c - b;

    // 利用向量叉乘，获得第一个基向量的 xyz 表示
    Vector3d k = ba.cross(bc);
    k.normalize();

    // 将 ba 向量作为 第二个基向量的 xyz 表示
    Vector3d j = ba;
    j.normalize();

    // 利用向量叉乘求得第三个基向量的 xyz 表示
    Vector3d i = k.cross(j);
    i.normalize();

    /// 获得转换矩阵
    Matrix3d transMat = Matrix3d::Zero();
    transMat << i, j, k;

    // 转置
    transMat.transpose();

    return transMat;
}
