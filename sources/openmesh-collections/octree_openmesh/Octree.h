#ifndef OCTREE_H
#define OCTREE_H

#include <vector>
#include <set>
#include <cmath>
#include <list>
#include <stack>
#include <memory>

#include <Eigen/Dense>

#include "Mesh.h"
#include "BoundingBoxHelper.h"
#include "primitives.h"
//#include "Triaccel.h"

using Eigen::Vector3d;
using Eigen::AlignedBox3d;
using std::vector;
using std::stack;
using std::shared_ptr;
using IndexSet = std::set<int>;
using IndexSetIter = IndexSet::iterator;

//#define USE_TRI_ACCEL 1
const int MAX_DEPTH = 8;
const int DEFAULT_OCTREE_NODE_SIZE = 64;

class Octree
{
public:

    // 包围盒(会比原始模型有一定程度的放大
    AlignedBox3d boundingBox;

    // 原始包围盒
    AlignedBox3d originBoundingBox;

    // 子包围盒
    vector<Octree> children;

    // 面片数据
    vector<Mesh::FaceHandle> triangleData;

    // 指向父节点的指针
    Octree *parent;

    // 网格模型
    shared_ptr<Mesh> mesh;

    // 每个 OCtree 最多三角面片数量
    int trianglePerNode;

    // 空构造函数
    Octree();

    // 构造函数
    Octree(Mesh& mesh, int triPerNode = DEFAULT_OCTREE_NODE_SIZE);
    Octree(int triPerNode, const AlignedBox3d& bb, const std::vector<Mesh::FaceHandle>& tris, shared_ptr<Mesh> useMesh);

    // 射线最近相交点
    Vector3d closestIntersectionPoint(const Eigen::ParametrizedLine<double,3>& eigenRay, int* faceIndex);
    Vector3d closestIntersectionPoint(const Ray &ray, int *faceIndex, bool isRobsut = false);

    bool isIntersectsWithRay(const Ray &ray, double *distance);
    IndexSet intersectSphere(const Vector3d &sphere_center, double radius);
    inline shared_ptr<Mesh> getMesh(){return mesh;}

    // 获得顶点包围盒所在的所有面片
    IndexSet intersectPoint(const Vector3d& point);

    // 迭代获得顶点所在包围盒的所有面片
    void intersectRecursivePoint(const Vector3d& point, IndexSet& tris);

    // 如果是叶子节点，将叶子节点的所有面片加入集合
    bool intersectHit(IndexSet& triangleSet);

    // 找到射线相交的所有面片
    IndexSet intersectRay(Ray ray, double rayThickness, bool isFullTest) const;

    // 递归的将所有有交集的叶子节点的面片塞入集合
    void intersectRecursiveRay(const Ray& ray, IndexSet& triangleSet);

    void intersectRecursiveSphere(const Eigen::Vector3d& sphere_center, double radius, IndexSet& triangleSet);

    // 测试射线是否与面片相交
    bool testIntersectHit(const Ray& ray, HitResult & hitResult);

    Octree* root()
    {
        if(parent == nullptr){
            return this;
        }else {
            return parent->root();
        }
    }

    // 根据面片获得三个顶点
    vector<Eigen::Vector3d> triPoints(Mesh::FaceHandle f) const;

    // 面片与三角形面片的相交测试
    void rayTriangleIntersectionTest( Mesh::FaceHandle faceIndex, const Ray & ray, HitResult & hitResult, bool allowBack = true) const;

private:

    // 初始化构建
    void initBuild(vector<Mesh::FaceHandle>& tris, int triPerNode);

    vector<Mesh::FaceHandle> getIntersectingTris(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2, bool showIt);

    // 构建
    void build(int depth = 0);

    // 构建新节点
    void newNode(int depth, double x, double y, double z);

};

#endif // OCTREE_H
