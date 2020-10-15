#include "BoundingBoxHelper.h"

Eigen::AlignedBox3d computeFromMesh(Mesh &mesh)
{
    assert(mesh.n_vertices() >0);

    double minx = 0, miny = 0, minz = 0;
    double maxx = 0, maxy = 0, maxz = 0;

    Mesh::Point tmpPoint = mesh.point(mesh.vertex_handle(0));

    minx = maxx = tmpPoint.x();
    miny = maxy = tmpPoint.y();
    minz = maxz = tmpPoint.z();

    for(Mesh::VIter viter = mesh.vertices_begin(); viter != mesh.vertices_end(); viter++)
    {
        tmpPoint = mesh.point(*viter);

        minx = std::min(tmpPoint.x(), minx);
        miny = std::min(tmpPoint.y(), miny);
        minz = std::min(tmpPoint.z(), minz);

        maxx = std::max(tmpPoint.x(), maxx);
        maxy = std::max(tmpPoint.y(), maxy);
        maxz = std::max(tmpPoint.z(), maxz);
    }

    Vector3d vmax(maxx, maxy, maxz);
    Vector3d vmin(minx, miny, minz);

    AlignedBox3d boundingBox;
    boundingBox.extend(vmax);
    boundingBox.extend(vmin);

    return boundingBox;
}

bool intersects(const Eigen::AlignedBox3d &boundingBox, const Ray &ray)
{
    // r.dir is unit direction vector of ray
    Eigen::Vector3d dirfrac;
    dirfrac.x() = 1.0f / ray.direction.x();
    dirfrac.y() = 1.0f / ray.direction.y();
    dirfrac.z() = 1.0f / ray.direction.z();

    Vector3d vmin = boundingBox.min();
    Vector3d vmax = boundingBox.max();
    Vector3d center = boundingBox.center();

    double overlap = ray.thickness;
    Eigen::Vector3d minv = vmin + ((vmin - center).normalized() * overlap);
    Eigen::Vector3d maxv = vmax + ((vmax - center).normalized() * overlap);

    // lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
    // ray.origin is origin of ray
    float t1 = (minv.x() - ray.origin.x())*dirfrac.x();
    float t2 = (maxv.x() - ray.origin.x())*dirfrac.x();
    float t3 = (minv.y() - ray.origin.y())*dirfrac.y();
    float t4 = (maxv.y() - ray.origin.y())*dirfrac.y();
    float t5 = (minv.z() - ray.origin.z())*dirfrac.z();
    float t6 = (maxv.z() - ray.origin.z())*dirfrac.z();

    float tmin = std::max(
                     std::max(std::min(t1, t2),std::min(t3, t4)),
                     std::min(t5, t6));
    float tmax = std::min(
                     std::min(std::max(t1, t2), std::max(t3, t4)),
                     std::max(t5, t6));

    // if tmax < 0, ray (line) is intersecting AABB, but whole AABB is behing us
    if (tmax < 0)
    {
        return false;
    }

    // if tmin > tmax, ray doesn't intersect AABB
    if (tmin > tmax)
    {
        return false;
    }

    return true;
}

bool intersectsWorking(const Eigen::AlignedBox3d &boundingBox, const Ray& ray)
{
    // vectors to hold the T-values for every direction
    Eigen::Vector3d T_1, T_2;

    // maximums defined in float.h
    double t_near = -DBL_MAX;
    double t_far = DBL_MAX;

    Vector3d vmin = boundingBox.min();
    Vector3d vmax = boundingBox.max();

    //we test slabs in every direction
    for (int i = 0; i < 3; i++){
        // ray parallel to planes in this direction
        if (ray.direction[i] == 0){
            if ((ray.origin[i] < vmin[i]) || (ray.origin[i] > vmax[i])) {
                // parallel AND outside box : no intersection possible
                return false;
            }
        } else {
            // ray not parallel to planes in this direction

            T_1[i] = (vmin[i] - ray.origin[i]) / ray.direction[i];
            T_2[i] = (vmax[i] - ray.origin[i]) / ray.direction[i];

            // we want T_1 to hold values for intersection with near plane
            if(T_1[i] > T_2[i]){
                std::swap(T_1,T_2);
            }
            if (T_1[i] > t_near){
                t_near = T_1[i];
            }
            if (T_2[i] < t_far){
                t_far = T_2[i];
            }
            if( (t_near > t_far) || (t_far < 0) ){
                return false;
            }
        }
    }

    // if we made it here, there was an intersection - YAY
    return true;
}

bool intersectsOld(const Eigen::AlignedBox3d &boundingBox, const Ray &ray)
{
    double rhs;
    double fWdU[3];
    double fAWdU[3];
    double fDdU[3];
    double fADdU[3];
    double fAWxDdU[3];

    Vector3d vmin = boundingBox.min();
    Vector3d vmax = boundingBox.max();
    Vector3d center = boundingBox.center();

    double xExtent = vmax.x() - center.x();
    double yExtent = vmax.y() - center.y();
    double zExtent = vmax.z() - center.z();

    Eigen::Vector3d UNIT_X(1.0, 0.0, 0.0);
    Eigen::Vector3d UNIT_Y(0.0, 1.0, 0.0);
    Eigen::Vector3d UNIT_Z(0.0, 0.0, 1.0);

    Eigen::Vector3d diff = ray.origin - center;
    Eigen::Vector3d wCrossD = cross(ray.direction , diff);

    fWdU[0] = dot(ray.direction , UNIT_X);
    fAWdU[0] = fabs(fWdU[0]);
    fDdU[0] = dot(diff , UNIT_X);
    fADdU[0] = fabs(fDdU[0]);
    if (fADdU[0] > xExtent && fDdU[0] * fWdU[0] >= 0.0)        return false;

    fWdU[1] = dot(ray.direction , UNIT_Y);
    fAWdU[1] = fabs(fWdU[1]);
    fDdU[1] = dot(diff , UNIT_Y);
    fADdU[1] = fabs(fDdU[1]);
    if (fADdU[1] > yExtent && fDdU[1] * fWdU[1] >= 0.0)        return false;

    fWdU[2] = dot(ray.direction , UNIT_Z);
    fAWdU[2] = fabs(fWdU[2]);
    fDdU[2] = dot(diff , UNIT_Z);
    fADdU[2] = fabs(fDdU[2]);
    if (fADdU[2] > zExtent && fDdU[2] * fWdU[2] >= 0.0)        return false;

    fAWxDdU[0] = fabs(dot(wCrossD , UNIT_X));
    rhs = yExtent * fAWdU[2] + zExtent * fAWdU[1];
    if (fAWxDdU[0] > rhs)        return false;

    fAWxDdU[1] = fabs(dot(wCrossD , UNIT_Y));
    rhs = xExtent * fAWdU[2] + zExtent * fAWdU[0];
    if (fAWxDdU[1] > rhs)        return false;

    fAWxDdU[2] = fabs(dot(wCrossD , UNIT_Z));
    rhs = xExtent * fAWdU[1] + yExtent * fAWdU[0];
    if (fAWxDdU[2] > rhs)        return false;

    return true;
}

/* AABB-triangle overlap test code                      */
/* by Tomas Akenine                            */
/**
 * @brief containsTriangle
 * @param boundingBox
 * @param tv0
 * @param tv1
 * @param tv2
 * @return
 */
bool containsTriangle(
        const Eigen::AlignedBox3d &boundingBox,
        const Eigen::Vector3d &tv0,
        const Eigen::Vector3d &tv1,
        const Eigen::Vector3d &tv2)
{
    Vector3d boxcenter = boundingBox.center();
    Vector3d vmax = boundingBox.max();

    double xExtent = vmax.x() - boxcenter.x();
    double yExtent = vmax.y() - boxcenter.y();
    double zExtent = vmax.z() - boxcenter.z();

    Vector3d boxhalfsize(xExtent, yExtent, zExtent);

    int X = 0, Y = 1, Z = 2;

    /*    use separating axis theorem to test overlap between triangle and box */
    /*    need to test for overlap in these directions: */
    /*    1) the {x,y,z}-directions (actually, since we use the AABB of the triangle */
    /*       we do not even need to test these) */
    /*    2) normal of the triangle */
    /*    3) crossproduct(edge from tri, {x,y,z}-directin) */
    /*       this gives 3x3=9 more tests */
    Eigen::Vector3d v0,v1,v2;
    double min,max,p0,p1,p2,rad,fex,fey,fez;
    Eigen::Vector3d normal,e0,e1,e2;

    /* This is the fastest branch on Sun */
    /* move everything so that the box center is in (0,0,0) */
    v0=tv0-boxcenter;
    v1=tv1-boxcenter;
    v2=tv2-boxcenter;

    /* compute triangle edges */
    e0=v1-v0;      /* tri edge 0 */
    e1=v2-v1;      /* tri edge 1 */
    e2=v0-v2;      /* tri edge 2 */

    /* Bullet 3:  */
    /*  test the 9 tests first (this was faster) */
    fex = fabsf(e0[X]);
    fey = fabsf(e0[Y]);
    fez = fabsf(e0[Z]);
    AXISTEST_X01(e0[Z], e0[Y], fez, fey);
    AXISTEST_Y02(e0[Z], e0[X], fez, fex);
    AXISTEST_Z12(e0[Y], e0[X], fey, fex);
    fex = fabsf(e1[X]);
    fey = fabsf(e1[Y]);
    fez = fabsf(e1[Z]);
    AXISTEST_X01(e1[Z], e1[Y], fez, fey);
    AXISTEST_Y02(e1[Z], e1[X], fez, fex);
    AXISTEST_Z0(e1[Y], e1[X], fey, fex);
    fex = fabsf(e2[X]);
    fey = fabsf(e2[Y]);
    fez = fabsf(e2[Z]);
    AXISTEST_X2(e2[Z], e2[Y], fez, fey);
    AXISTEST_Y1(e2[Z], e2[X], fez, fex);
    AXISTEST_Z12(e2[Y], e2[X], fey, fex);

    /* Bullet 1: */
    /*  first test overlap in the {x,y,z}-directions */
    /*  find min, max of the triangle each direction, and test for overlap in */
    /*  that direction -- this is equivalent to testing a minimal AABB around */
    /*  the triangle against the AABB */
    /* test in X-direction */
    findMinMax(v0[X],v1[X],v2[X],min,max);
    if(min>boxhalfsize[X] || max<-boxhalfsize[X]) return 0;
    /* test in Y-direction */
    findMinMax(v0[Y],v1[Y],v2[Y],min,max);
    if(min>boxhalfsize[Y] || max<-boxhalfsize[Y]) return 0;
    /* test in Z-direction */
    findMinMax(v0[Z],v1[Z],v2[Z],min,max);
    if(min>boxhalfsize[Z] || max<-boxhalfsize[Z]) return 0;

    /* Bullet 2: */
    /*  test if the box intersects the plane of the triangle */
    /*  compute plane equation of triangle: normal*x+d=0 */
    normal = cross(e0, e1);

    if(!planeBoxOverlap(normal,v0,boxhalfsize)) return 0;
    return 1;   /* box and triangle overlaps */
}

bool intersectsSphere(
        const Eigen::AlignedBox3d &boundingBox,
        const Eigen::Vector3d &sphere_center,
        double radius)
{
    Vector3d boxcenter = boundingBox.center();
    Vector3d vmax = boundingBox.max();

    double xExtent = vmax.x() - boxcenter.x();
    double yExtent = vmax.y() - boxcenter.y();
    double zExtent = vmax.z() - boxcenter.z();

    if (fabs(boxcenter.x() - sphere_center.x()) < radius + xExtent
            && fabs(boxcenter.y() - sphere_center.y()) < radius + yExtent
            && fabs(boxcenter.z() - sphere_center.z()) < radius + zExtent)
        return true;

    return false;
}
