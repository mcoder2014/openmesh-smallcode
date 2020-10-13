#include "Projection.h"

Projection::Projection()
{

}

Mesh Projection::project(Mesh &mesh, vector<Eigen::Vector3d> face)
{

}

Mesh Projection::ProjectZ(Mesh &mesh)
{
    Mesh dst;
    dst.assign(mesh);

    for(Mesh::VIter viter = dst.vertices_begin(); viter!=dst.vertices_end(); viter++)
    {
        Mesh::Point& point = dst.point(*viter);
        point.z() = 0;
    }

    return dst;
}
