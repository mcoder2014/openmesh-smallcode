#include "LaplaceDeformation.h"

#include <assert.h>

void LaplaceDeformation::initMatrix()
{
    buildMatrixLsTLs();
    buildMatrixLsTB();
}

/**
 * @brief LaplaceTransformation::buildMatrixLsTLs
 * 构造矩阵 LsT * Ls
 */
void LaplaceDeformation::buildMatrixLsTLs()
{
    int numN = static_cast<int>(sourceMesh.n_vertices());
    int numM = static_cast<int>(this->fixedAnchorIndex.size() + this->moveAnchorIndex.size());

    matrixL = MatrixXf(numN, numN);
    MatrixXf matrixLs(numM + numN, numN);

    // init zero matrix
    for(int i = 0; i < numN; i++) {
        for(int j = 0; j < numN; j++) {
            matrixL(i, j) = 0;
            matrixLs(i, j) = 0;
        }
    }
    for(int i = numN; i < numN + numM; i++) {
        for(int j = 0; j < numN; j++) {
            matrixLs(i, j) = 0;
        }
    }

    /// N * N 部分
    for(int i = 0; i < numN; i++)
    {
        // 对角线
        matrixL(i, i) = degreeMap[i];
        matrixLs(i, i) = degreeMap[i];

        // 联通情况
        Mesh::VertexHandle vhandle = sourceMesh.vertex_handle(static_cast<uint>(i));
        for(Mesh::VVIter vviter = sourceMesh.vv_begin(vhandle);
                vviter != sourceMesh.vv_end(vhandle); vviter++)
        {
            matrixL(i, (*vviter).idx()) = -1;
            matrixLs(i, (*vviter).idx()) = -1;
        }
    }

    /// M * N 部分
    // 固定锚点
    int numBase = numN;
    for(size_t i = 0; i < fixedAnchorIndex.size(); i++)
    {
        matrixLs(numBase + static_cast<int>(i), fixedAnchorIndex[i]) = 1;
    }

    // 移动锚点
    numBase = numN + static_cast<int>(fixedAnchorIndex.size());
    for(size_t i = 0; i < moveAnchorIndex.size(); i++)
    {
        matrixLs(numBase + static_cast<int>(i), moveAnchorIndex[i]) = 1;
    }

    matrixLsT = matrixLs.transpose();
    matrixLsTLs = matrixLsT * matrixLs;

}

/**
 * @brief LaplaceTransformation::buildMatrixLsTB
 * 构造矩阵 LsT * B
 */
void LaplaceDeformation::buildMatrixLsTB()
{
    int numN = static_cast<int>(sourceMesh.n_vertices());
    int numM = static_cast<int>(this->fixedAnchorIndex.size() + this->moveAnchorIndex.size());

    // 初始化矩阵
    pointsX = MatrixXf(numN, 1);
    pointsY = MatrixXf(numN, 1);
    pointsZ = MatrixXf(numN, 1);

    for(Mesh::VIter viter = sourceMesh.vertices_begin();
            viter!=sourceMesh.vertices_end(); viter++)
    {
        int index = (*viter).idx();
        Mesh::Point point = sourceMesh.point(*viter);

        pointsX(index, 0) = static_cast<float>(point.x());
        pointsY(index, 0) = static_cast<float>(point.y());
        pointsZ(index, 0) = static_cast<float>(point.z());
    }

    // 计算 B 矩阵初始值
    MatrixXf matrixBX = matrixL * pointsX;
    MatrixXf matrixBY = matrixL * pointsY;
    MatrixXf matrixBZ = matrixL * pointsZ;

    matrixBX.conservativeResize(numM + numN, 1);
    matrixBY.conservativeResize(numM + numN, 1);
    matrixBZ.conservativeResize(numM + numN, 1);

    // 赋值锚点坐标
    // 固定锚点
    int numBase = numN;
    for(size_t i = 0; i <fixedAnchorIndex.size(); i++)
    {
        Mesh::Point point = sourceMesh.point(
            sourceMesh.vertex_handle(static_cast<uint>(fixedAnchorIndex[i])));
        matrixBX(numBase + static_cast<int>(i), 0) = static_cast<float>(point.x());
        matrixBY(numBase + static_cast<int>(i), 0) = static_cast<float>(point.y());
        matrixBZ(numBase + static_cast<int>(i), 0) = static_cast<float>(point.z());
    }

    // 移动锚点
    numBase = numN + static_cast<int>(fixedAnchorIndex.size());
    for(size_t i = 0; i < moveAnchorIndex.size(); i++)
    {
        Mesh::Point point = moveAnchorPosition[i];
        matrixBX(numBase + static_cast<int>(i), 0) = static_cast<float>(point.x());
        matrixBY(numBase + static_cast<int>(i), 0) = static_cast<float>(point.y());
        matrixBZ(numBase + static_cast<int>(i), 0) = static_cast<float>(point.z());
    }

    // 计算 LsTB
    matrixLsTBX = matrixLsT * matrixBX;
    matrixLsTBY = matrixLsT * matrixBY;
    matrixLsTBZ = matrixLsT * matrixBZ;
}

/**
 * @brief LaplaceTransformation::calculate
 * 解线性方程 LsT*Ls * X = LsT * B
 */
void LaplaceDeformation::calculate()
{

    Eigen::HouseholderQR<MatrixXf> qr;
    qr.compute(matrixLsTLs);
    pointsX = qr.solve(matrixLsTBX);
    pointsY = qr.solve(matrixLsTBY);
    pointsZ = qr.solve(matrixLsTBZ);

}

/**
 * @brief LaplaceTransformation::applyTransformation
 * 把计算结果，构造目标模型的网格
 * @return
 */
Mesh LaplaceDeformation::applyTransformation()
{
    int numN = static_cast<int>(sourceMesh.n_vertices());

    // 应用结果
    Mesh dstMesh = sourceMesh;
    Mesh::Point tmpPoint;

    for(int i = 0; i < numN; i++)
    {
        tmpPoint.x() = static_cast<double>(pointsX(i, 0));
        tmpPoint.y() = static_cast<double>(pointsY(i, 0));
        tmpPoint.z() = static_cast<double>(pointsZ(i, 0));

        dstMesh.point(dstMesh.vertex_handle(static_cast<uint>(i))) = tmpPoint;
    }

    return dstMesh;
}

void LaplaceDeformation::getDegree()
{
    degreeMap.clear();

    for(Mesh::VertexIter viter = sourceMesh.vertices_begin();
            viter != sourceMesh.vertices_end(); viter++)
    {
        // 顶点的 idx
        int index = (*viter).idx();
        int degree = 0;
        for(Mesh::VertexVertexIter vviter = sourceMesh.vv_begin(*viter);
                vviter.is_valid(); vviter++)
        {
            degree++;
        }
        degreeMap[index] = degree;
    }
}

void LaplaceDeformation::setMoveAnchors(vector<int> &moveAnchorIndex, vector<Eigen::Vector3d> &moveAnchorPosition)
{
    assert(moveAnchorIndex.size() == moveAnchorPosition.size());

    this->moveAnchorIndex.clear();
    this->moveAnchorIndex.assign(moveAnchorIndex.begin(), moveAnchorIndex.end());
    this->moveAnchorPosition.clear();
    this->moveAnchorPosition.assign(moveAnchorPosition.begin(), moveAnchorPosition.end());
}

void LaplaceDeformation::addMoveAnchor(int moveIndex, Eigen::Vector3d position)
{
    this->moveAnchorIndex.push_back(moveIndex);
    this->moveAnchorPosition.push_back(position);
}

void LaplaceDeformation::setFixedAnchors(vector<int> &fixedAnchorIndex)
{
    this->fixedAnchorIndex.clear();
    this->fixedAnchorIndex.assign(fixedAnchorIndex.begin(), fixedAnchorIndex.end());
}

void LaplaceDeformation::addFixedAnchor(int fixedIndex)
{
    this->fixedAnchorIndex.push_back(fixedIndex);
}

/**
 * @brief LaplaceTransformation::apply
 * 实施拉普拉斯变化
 * @return
 */
Mesh LaplaceDeformation::apply()
{
    // 先预计算模型每个顶点的度
    getDegree();

    // 初始化矩阵
    initMatrix();

    // 计算结果
    calculate();

    return applyTransformation();
}
