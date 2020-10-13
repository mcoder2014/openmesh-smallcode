#ifndef PROJECTION_H
#define PROJECTION_H

#include <vector>

#include <Eigen/Dense>

#include "Mesh.h"

using std::vector;
using Eigen::Vector3d;

class Projection
{
public:
    Projection();

    // 投影在 三点构成的面上
    Mesh project(Mesh& mesh, vector<Vector3d> face);

    // 投影在 XoY 平面上
    Mesh ProjectZ(Mesh& mesh);
};

#endif // PROJECTION_H
