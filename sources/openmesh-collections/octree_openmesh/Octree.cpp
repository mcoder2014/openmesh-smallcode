#include "Octree.h"

#include <iostream>

/**
 * @brief Octree::Octree
 */
Octree::Octree()
{
    trianglePerNode = -1;
    parent = nullptr;
    mesh = nullptr;
}

/**
 * @brief Octree::Octree
 * 最经常使用的构造函数
 * @param mesh
 * @param triPerNode
 */
Octree::Octree(Mesh &mesh, int triPerNode)
{
    this->parent = nullptr;

    // 为八叉树做一个备份 mesh，不影响到原 mesh
    this->mesh = std::make_shared<Mesh>();
    this->mesh->assign(mesh);

    this->trianglePerNode = triPerNode;

    // 记录下所有面片
    std::vector<Mesh::FaceHandle> allTris;
    for(Mesh::FaceIter fiter = this->mesh->faces_begin();
        fiter != this->mesh->faces_end(); fiter ++){
        allTris.push_back(*fiter);
    }

    // 初始化整个八叉树
    this->initBuild(allTris, trianglePerNode);
}

Octree::Octree(
        int triPerNode,
        const Eigen::AlignedBox3d &bb,
        const std::vector<Mesh::FaceHandle> &tris,
        shared_ptr<Mesh> useMesh)
{
    this->parent = nullptr;
    this->mesh = useMesh;
    this->boundingBox = bb;
    this->trianglePerNode = triPerNode;
    this->triangleData = tris;
}

/**
 * @brief Octree::closestIntersectionPoint
 * @param ray
 * @param faceIndex
 * @return
 */
Eigen::Vector3d Octree::closestIntersectionPoint(const Eigen::ParametrizedLine<double,3>& eigenRay, int *faceIndex)
{
    Ray ray(eigenRay.origin(), eigenRay.direction());
    return closestIntersectionPoint(ray, faceIndex);
}

/**
 * @brief Octree::closestIntersectionPoint
 * @param ray
 * @param faceIndex
 * @param isRobsut
 * @return
 */
Eigen::Vector3d Octree::closestIntersectionPoint(const Ray &ray, int *faceIndex, bool isRobsut)
{
    HitResult res, best_res;
    Eigen::Vector3d intersectPoint(0.0,0.0,0.0);
    double minDistance = DBL_MAX;
    if(faceIndex) {
        *faceIndex = -1;
    }

    for( int idx : intersectRay( ray, 0.01, false ) ){
        rayTriangleIntersectionTest(mesh->face_handle(static_cast<uint>(idx)), ray, res, true);
        if(res.hit){
            if (res.distance < minDistance){
                minDistance = res.distance;
                intersectPoint = ray.origin + (ray.direction * res.distance);
                if(faceIndex) {
                    *faceIndex = idx;
                }
                best_res = res;
            }
        }
    }

    return intersectPoint;
}

/**
 * @brief Octree::isIntersectsWithRay
 * 是否与射线相交
 * @param ray
 * @param distance
 * @return
 */
bool Octree::isIntersectsWithRay(const Ray &ray, double *distance)
{
    int faceIndex = -1;
    if(distance) {
        *distance = DBL_MAX;
    }

    Vector3d intersectPoint = closestIntersectionPoint(ray, &faceIndex);

    if(faceIndex >= 0){
        if(distance){
            *distance = (intersectPoint - ray.origin).norm();
        }
        return true;
    }
    return false;
}

/**
 * @brief Octree::intersectSphere
 * 判断是否和球体相交
 * @param sphere_center
 * @param radius
 * @return
 */
IndexSet Octree::intersectSphere(const Eigen::Vector3d &sphere_center, double radius)
{
    IndexSet tris;
    if(intersectsSphere(boundingBox, sphere_center, radius))
    {
        intersectRecursiveSphere(sphere_center, radius, tris);
    }
    return tris;
}

/**
 * @brief Octree::intersectPoint
 * 获得顶点所在包围盒的所有面片
 * @param point
 * @return
 */
IndexSet Octree::intersectPoint(const Eigen::Vector3d &point)
{
    IndexSet indexSet;

    if (boundingBox.contains(point))
        intersectRecursivePoint(point, indexSet);

    return indexSet;
}

/**
 * @brief Octree::intersectRecursivePoint
 * 迭代获得顶点所在包围盒的所有面片
 * @param point
 * @param tris
 */
void Octree::intersectRecursivePoint(const Eigen::Vector3d &point, IndexSet &tris)
{
    // 和当前包围盒相交，返回
    if (intersectHit(tris))
        return;

    for (std::vector<Octree>::iterator child = children.begin();  child != children.end(); child++)
    {
        if (child->boundingBox.contains(point))
            child->intersectRecursivePoint(point, tris);
    }
}

/**
 * @brief Octree::intersectHit
 * 如果是叶子节点，将叶子节点的所有面片加入集合
 * @param triangleSet
 * @return
 */
bool Octree::intersectHit(IndexSet &triangleSet)
{
    if( this->children.size() > 0 )
        return false;

    for(Mesh::FaceHandle faceHandle : triangleData)
    {
        triangleSet.insert(faceHandle.idx());
    }

    return true;
}

/**
 * @brief Octree::intersectRay
 * 找到射线相交的所有面片
 * @param ray
 * @param rayThickness
 * @param isFullTest
 * @return
 */
IndexSet Octree::intersectRay(Ray ray, double rayThickness, bool isFullTest) const
{
    IndexSet triangleSet;
    ray.thickness = rayThickness;

    if (intersects(boundingBox, ray))
    {
        stack<const Octree*> octreeStack;
        octreeStack.push(this);

        while( !octreeStack.empty() )
        {
            const Octree * curTree = octreeStack.top();
            octreeStack.pop();

            if(curTree->children.size() == 0)
            {
                // 把最后一层的相交的八叉树盒子中所有面片加入 三角形集合
                for(Mesh::FaceHandle faceHandle : curTree->triangleData){
                    triangleSet.insert(faceHandle.idx());
                }
            }

            // Do following if child size > 0
            // 如果子节点和射线相交，则后续对子节点进行处理
            for (std::vector<Octree>::const_iterator child = curTree->children.begin();  child != curTree->children.end(); child++)
            {
                if(intersects(child->boundingBox,ray) ) {
                    octreeStack.push( &(*child) );
                }
            }
        }

        if(isFullTest)
        {
            // 是否全测试
            IndexSet exactSet;
            for(int faceIndex : triangleSet)
            {
                HitResult hitRes;
                rayTriangleIntersectionTest(mesh->face_handle(static_cast<uint>(faceIndex)), ray, hitRes, false);
                if(hitRes.hit) exactSet.insert(faceIndex);
            }
            return exactSet;
        }
    }

    // debug
//    std::cout << "set size " << triangleSet.size() << std::endl;
//    for(int i : triangleSet) {
//        std::cout << i << "\t";
//    }
//    std::cout << std::endl;

    return triangleSet;
}

/**
 * @brief Octree::intersectRecursiveRay
 * 递归的将所有交集的叶子节点的面片塞入集合
 * @param ray
 * @param triangleSet
 */
void Octree::intersectRecursiveRay(const Ray &ray, IndexSet &triangleSet)
{
    if(children.size() == 0){
        for(Mesh::FaceHandle faceHandle:triangleData){
            triangleSet.insert(faceHandle.idx());
        }
    }

    // Do following if child size > 0
    for (std::vector<Octree>::iterator child = children.begin();  child != children.end(); child++){
        if (intersects(child->boundingBox, ray) ){
            child->intersectRecursiveRay(ray, triangleSet);
        }
    }
}

/**
 * @brief Octree::intersectRecursiveSphere
 * 递归的和球体求交点
 * @param sphere_center
 * @param radius
 * @param triangleSet
 */
void Octree::intersectRecursiveSphere(const Eigen::Vector3d &sphere_center, double radius, IndexSet &triangleSet)
{
    // Leaf node ?
    if (intersectHit(triangleSet))
        return;

    // Visist children
    for (std::vector<Octree>::iterator child = children.begin();  child != children.end(); child++)
    {
        if (intersectsSphere(child->boundingBox, sphere_center, radius))
            child->intersectRecursiveSphere(sphere_center, radius, triangleSet);
    }
}

/**
 * @brief Octree::testIntersectHit
 * @param ray
 * @param hitResult
 * @return
 */
bool Octree::testIntersectHit(const Ray &ray, HitResult &hitResult)
{
    if(this->children.size() > 0)
        return false;

    // Do actual intersection test
    for(Mesh::FaceHandle faceHandle:triangleData)
    {
        rayTriangleIntersectionTest(faceHandle, ray, hitResult, true);
        if(hitResult.hit)
            return true;
    }

    return false;
}

/**
 * @brief Octree::triPoints
 * 获得三角形的三个面片
 * @param f
 * @return
 */
vector<Eigen::Vector3d> Octree::triPoints(Mesh::FaceHandle faceHandle) const
{
    vector<Vector3d> points;
    for (Mesh::FaceVertexIter fviter = mesh->fv_iter(faceHandle);
         fviter != mesh->fv_end(faceHandle); fviter++) {
        Vector3d point = mesh->point(*fviter);
        points.push_back(point);
    }
    return points;
}

/**
 * @brief Octree::rayTriangleIntersectionTest
 * @param faceIndex
 * @param ray
 * @param hitResult
 * @param allowBack
 */
void Octree::rayTriangleIntersectionTest(Mesh::FaceHandle faceIndex, const Ray &ray, HitResult &hitResult, bool allowBack) const
{
    hitResult.hit = false;
    hitResult.distance = DBL_MAX;

    // 精度值
    double EPS = 1e-7;

    std::vector<Eigen::Vector3d> v = triPoints(faceIndex);

    Eigen::Vector3d vertex1 = v[0];
    Eigen::Vector3d vertex2 = v[1];
    Eigen::Vector3d vertex3 = v[2];

    // Compute vectors along two edges of the triangle.
    Eigen::Vector3d edge1 = vertex2 - vertex1;
    Eigen::Vector3d edge2 = vertex3 - vertex1;

    // Compute the determinant.
    Eigen::Vector3d directionCrossEdge2 = cross(ray.direction, edge2);

    double determinant = dot(edge1, directionCrossEdge2);

    // If the ray is parallel to the triangle plane, there is no collision.
    if (fabs(determinant) < EPS)
        return;

    double inverseDeterminant = 1.0 / determinant;

    // Calculate the U parameter of the intersection point.
    Eigen::Vector3d distVector = ray.origin - vertex1;
    double triangleU = dot(distVector, directionCrossEdge2);
    triangleU *= inverseDeterminant;

    // Make sure it is inside the triangle.
    if (triangleU < 0 - EPS || triangleU > 1 + EPS)
        return;

    // Calculate the V parameter of the intersection point.
    Eigen::Vector3d distanceCrossEdge1 = cross(distVector, edge1);
    double triangleV = dot(ray.direction, distanceCrossEdge1);
    triangleV *= inverseDeterminant;

    // Make sure it is inside the triangle.
    if (triangleV < 0 - EPS || triangleU + triangleV > 1 + EPS)
        return;

    // Compute the distance along the ray to the triangle.
    double rayDistance = dot(edge2, distanceCrossEdge1);
    rayDistance *= inverseDeterminant;

    if(!allowBack){
        // Is the triangle behind the ray origin?
        if (rayDistance < 0)
            return;
    }

    hitResult.hit = true;
    hitResult.distance = rayDistance;

    hitResult.u = triangleU;
    hitResult.v = triangleV;

    hitResult.index = faceIndex.idx();
}

/**
 * @brief Octree::initBuild
 * @param tris
 * @param triPerNode
 */
void Octree::initBuild(vector<Mesh::FaceHandle> &tris, int triPerNode)
{
    /// 加入三角面片
    this->triangleData = tris;
    this->trianglePerNode = triPerNode;

    // 创建包围盒
    AlignedBox3d bigBoundingBox = getBoundingBoxFromMesh(*mesh);
    // 备份模型原始包围盒大小
    originBoundingBox = bigBoundingBox;

    Vector3d extent = bigBoundingBox.max() - bigBoundingBox.min();

    // Transform and scale to node's coordinates
    double largeSize = std::max(extent.x(), std::max(extent.y(), extent.z()));
    largeSize *= 1.25;
    largeSize /= 2;

    Vector3d vcenter = bigBoundingBox.center();
    Vector3d vmin = vcenter, vmax = vcenter;

    vmin.x() -= largeSize;
    vmin.y() -= largeSize;
    vmin.z() -= largeSize;

    vmax.x() += largeSize;
    vmax.y() += largeSize;
    vmax.z() += largeSize;

    boundingBox.setNull();
    boundingBox.extend(vmin);
    boundingBox.extend(vmax);

    // Build the tree
    this->build();

    // Connect children with parent
    std::stack<Octree*> childStack;
    childStack.push(this);
    while(!childStack.empty())
    {
        Octree * curr = childStack.top(); childStack.pop();
        for(size_t i = 0; i < curr->children.size(); i++)
        {
            curr->children[i].parent = curr;
            childStack.push(&curr->children[i]);
        }
    }
}

/**
 * @brief Octree::getIntersectingTris
 * @param v0
 * @param v1
 * @param v2
 * @param showIt
 * @return
 */
vector<Mesh::FaceHandle> Octree::getIntersectingTris(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, bool showIt)
{
    if(this->triangleData.size() == 0 || this->children.size() == 0)
        return this->triangleData;

    std::vector<Mesh::FaceHandle> res;

    for(size_t i = 0; i < this->children.size(); i++)
    {
        if(containsTriangle(children[i].boundingBox, v0, v1, v2))
        {
            const std::vector<Mesh::FaceHandle> tris = children[i].getIntersectingTris(v0, v1, v2, showIt);

            for(size_t j = 0; j < tris.size(); j++)
                res.push_back(tris[j]);
        }
    }

    return res;
}

/**
 * @brief Octree::build
 * 递归构建八叉树
 * @param depth
 */
void Octree::build(int depth)
{
    if (static_cast<int>(triangleData.size()) > this->trianglePerNode)
    {
        if(depth < MAX_DEPTH)
        {
            // Subdivide to 8 nodes
            newNode(depth, -1, -1, -1);
            newNode(depth, 1, -1, -1);
            newNode(depth, -1, 1, -1);
            newNode(depth, 1, 1, -1);
            newNode(depth, -1, -1, 1);
            newNode(depth, 1, -1, 1);
            newNode(depth, -1, 1, 1);
            newNode(depth, 1, 1, 1);
        }
    }
}

/**
 * @brief Octree::newNode
 * @param depth
 * @param x
 * @param y
 * @param z
 */
void Octree::newNode(int depth, double x, double y, double z)
{
    /// 构建 新节点的包围盒
    Vector3d extent = (boundingBox.max() - boundingBox.min()) / 2;
    Vector3d corner = boundingBox.center();
    corner.x() += extent.x() * x;
    corner.y() += extent.y() * y;
    corner.z() += extent.z() * z;

    AlignedBox3d bb;
    bb.extend(this->boundingBox.center());
    bb.extend(corner);

    /// Add child
    children.push_back(Octree());
    Octree &child = *(children.end()-1);

    child.mesh = mesh;
    child.boundingBox = bb;
    child.trianglePerNode = this->trianglePerNode;

    /// 筛选在子包围盒中的面片
    for(Mesh::FaceHandle faceHandle:triangleData)
    {
        vector<Vector3d> points = triPoints(faceHandle);
        if(containsTriangle(bb, points[0], points[1], points[2]))
        {
            child.triangleData.push_back(faceHandle);
        }
    }

    // 进行递归构建
    child.build(depth + 1);
}
