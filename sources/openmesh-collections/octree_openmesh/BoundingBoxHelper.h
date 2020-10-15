#ifndef BOUNDINGBOXHELPER_H
#define BOUNDINGBOXHELPER_H

#include <assert.h>
#include <climits>
#include <vector>

#include <Eigen/Dense>

#include "Mesh.h"
#include "primitives.h"

using Eigen::AlignedBox3d;
using Eigen::Vector3d;

// 从网格中计算包围盒
AlignedBox3d computeFromMesh(Mesh& mesh);

// 求包围盒和射线是否有交点
bool intersects(const Eigen::AlignedBox3d &boundingBox, const Ray& ray);

bool intersectsWorking(const Eigen::AlignedBox3d &boundingBox, const Ray& ray);

bool intersectsOld(const Eigen::AlignedBox3d &boundingBox, const Ray& ray);

bool containsTriangle(
        const Eigen::AlignedBox3d &boundingBox,
        const Eigen::Vector3d& tv0,
        const Eigen::Vector3d& tv1,
        const Eigen::Vector3d& tv2 );

bool intersectsSphere(const Eigen::AlignedBox3d &boundingBox, const Eigen::Vector3d& sphere_center, double radius );

#endif // BOUNDINGBOXHELPER_H
