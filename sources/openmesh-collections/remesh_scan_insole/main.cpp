#include <iostream>
#include <string>
#include <fstream>

#include <pcl/surface/gp3.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

#include "Mesh.h"

using namespace std;

/**
 * @brief triangle
 * 三角网格化
 * @param cloudWithNormal
 * @return
 */
pcl::PolygonMesh triangle(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{
    // Create search tree*
      pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
      tree2->setInputCloud (cloud);

      // Initialize objects
      pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
      pcl::PolygonMesh triangles;

      // Set the maximum distance between connected points (maximum edge length)
      gp3.setSearchRadius (5.0);

      // Set typical values for the parameters
      gp3.setMu (2.5);
      gp3.setMaximumNearestNeighbors (100);
      gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
      gp3.setMinimumAngle(M_PI/18); // 10 degrees
      gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
      gp3.setNormalConsistency(false);

      // Get result
      gp3.setInputCloud (cloud);
      gp3.setSearchMethod (tree2);
      gp3.reconstruct (triangles);

      return triangles;
}

/**
 * @brief loadPoints
 * @param fileName
 * @return
 */
pcl::PointCloud<pcl::PointNormal>::Ptr loadPoints(string fileName)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // 从文件中加载点云
    fstream fin(fileName);
    assert(fin.is_open());
    string tmp;
    while (getline(fin, tmp)) {
        std::istringstream ssIn(tmp);
        std::string tmpx[3];
        ssIn >> tmpx[0] >> tmpx[1] >> tmpx[2];
        pcl::PointXYZ point(std::stof(tmpx[0]), std::stof(tmpx[1]), std::stof(tmpx[2]));
        cloud->push_back(point);
    }
    fin.close();

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloudWithNormals);
    //* cloud_with_normals = cloud + normals

    return cloudWithNormals;
}

/**
 * @brief getMesh
 * @param pclMesh
 * @return
 */
Mesh getMesh(pcl::PolygonMesh& pclMesh, pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{
    Mesh dstMesh;
    Mesh::Point point;

    // 添加顶点
    for(pcl::PointCloud<pcl::PointNormal>::iterator iter = cloud->begin(); iter != cloud->end(); iter++)
    {
        point.x() = (*iter).x;
        point.y() = (*iter).y;
        point.z() = (*iter).z;
        dstMesh.add_vertex(point);
    }

    cout << "Faces count: " << pclMesh.polygons.size() << endl;

    // 添加面片
    vector<Mesh::VertexHandle> face;
    for (pcl::Vertices vertices:pclMesh.polygons) {
        face.clear();
        for(uint32_t index:vertices.vertices) {
            face.push_back(dstMesh.vertex_handle(index));
        }
        dstMesh.add_face(face);
    }

    return dstMesh;
}

int main(int argc, char* argv[])
{
    if(argc < 3) {
        cout << "pcl_triangle <points.txt> <destinationModel.ply>\n"
             << "support the file type as much as OpenMesh\n";
        return 0;
    }

    string sourceModelPath(argv[1]);
    string destModelPath(argv[2]);

    // load point cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud = loadPoints(sourceModelPath);

    // cloud triangle
    pcl::PolygonMesh pclMesh = triangle(cloud);

    // convert to openmesh
    Mesh destMesh = getMesh(pclMesh, cloud);

    // 保存
    OpenMesh::IO::Options option;
    option = OpenMesh::IO::Options::Binary;
    OpenMesh::IO::write_mesh(destMesh, destModelPath, option);

    return 0;
}
