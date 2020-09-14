#include "MeshAlgorithm.h"

#include <Eigen/Dense>

Eigen::AlignedBox3d getBoundingBox(Mesh& mesh)
{
    Eigen::AlignedBox3d boundingBox;

    for(Mesh::VertexIter iter = mesh.vertices_begin(); iter != mesh.vertices_end(); ++iter) {
        Mesh::Point point = mesh.point(*iter);
        boundingBox.extend(point);
    }

    return boundingBox;
}

/**
 * @brief mirror
 * @param mesh
 * @return
 */
Mesh mirror(Mesh &mesh)
{
    // 拷贝目标输出网格
    Mesh dest;
    dest.assign(mesh);

    // 找到对称面
    Eigen::AlignedBox3d boundingBox = getBoundingBox(mesh);
    Eigen::Vector3d center = boundingBox.center();
    double mid = center.x();

    // 镜像
    for(Mesh::VertexIter iter = dest.vertices_begin(); iter != dest.vertices_end(); ++iter) {
        Mesh::Point point = dest.point(*iter);
        point.x() = 2 * mid - point.x();
        dest.point(*iter) = point;
    }
    return dest;
}

/**
 * @brief revertFaceNormal
 * @param mesh
 */
Mesh revertFaceNormal(Mesh &mesh)
{
    Mesh dest;

    // 镜像
    for(Mesh::VertexIter iter = mesh.vertices_begin(); iter != mesh.vertices_end(); ++iter) {
        dest.add_vertex(mesh.point(*iter));
    }


    for(Mesh::FaceIter iter = mesh.faces_begin(); iter != mesh.faces_end(); iter++) {
        Mesh::FVIter fviter = mesh.fv_begin(*iter);
        Mesh::VertexHandle vh1 = *fviter;

        ++fviter;
        Mesh::VertexHandle vh2 = *fviter;

        ++fviter;
        Mesh::VertexHandle vh3 = *fviter;

        dest.add_face(vh3, vh2, vh1);
    }

    return dest;
}
