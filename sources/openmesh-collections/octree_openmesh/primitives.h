#ifndef PRIMITIVES_H
#define PRIMITIVES_H

#include <float.h>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <vector>

#define X 0
#define Y 1
#define Z 2

template<class T>
void findMinMax(const T& x0, const T& x1, const T& x2, T& min, T& max)
{
    min = std::min(x0, std::min(x1, x2));
    max = std::max(x0, std::max(x1, x2));
}

int planeBoxOverlap(const Eigen::Vector3d& normal, const Eigen::Vector3d& vert, const Eigen::Vector3d& maxbox);

/*======================== X-tests ========================*/
#define AXISTEST_X01(a, b, fa, fb)               \
    p0 = a*v0[Y] - b*v0[Z];                          \
    p2 = a*v2[Y] - b*v2[Z];                          \
    if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;} \
    rad = fa * boxhalfsize[Y] + fb * boxhalfsize[Z];   \
    if(min>rad || max<-rad) return 0;
#define AXISTEST_X2(a, b, fa, fb)               \
    p0 = a*v0[Y] - b*v0[Z];                       \
    p1 = a*v1[Y] - b*v1[Z];                          \
    if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
    rad = fa * boxhalfsize[Y] + fb * boxhalfsize[Z];   \
    if(min>rad || max<-rad) return 0;
/*======================== Y-tests ========================*/
#define AXISTEST_Y02(a, b, fa, fb)               \
    p0 = -a*v0[X] + b*v0[Z];                     \
    p2 = -a*v2[X] + b*v2[Z];                             \
    if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;} \
    rad = fa * boxhalfsize[X] + fb * boxhalfsize[Z];   \
    if(min>rad || max<-rad) return 0;
#define AXISTEST_Y1(a, b, fa, fb)               \
    p0 = -a*v0[X] + b*v0[Z];                     \
    p1 = -a*v1[X] + b*v1[Z];                           \
    if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
    rad = fa * boxhalfsize[X] + fb * boxhalfsize[Z];   \
    if(min>rad || max<-rad) return 0;
/*======================== Z-tests ========================*/
#define AXISTEST_Z12(a, b, fa, fb)               \
    p1 = a*v1[X] - b*v1[Y];                       \
    p2 = a*v2[X] - b*v2[Y];                          \
    if(p2<p1) {min=p2; max=p1;} else {min=p1; max=p2;} \
    rad = fa * boxhalfsize[X] + fb * boxhalfsize[Y];   \
    if(min>rad || max<-rad) return 0;
#define AXISTEST_Z0(a, b, fa, fb)               \
    p0 = a*v0[X] - b*v0[Y];                   \
    p1 = a*v1[X] - b*v1[Y];                       \
    if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
    rad = fa * boxhalfsize[X] + fb * boxhalfsize[Y];   \
    if(min>rad || max<-rad) return 0;

#undef X
#undef Y
#undef Z

class Ray
{
public:
    Eigen::Vector3d origin;
    Eigen::Vector3d direction;
    int index;
    double thickness;

    Ray(const Eigen::Vector3d & Origin = Eigen::Vector3d(),
        const Eigen::Vector3d & Direction = Eigen::Vector3d(),
        double Thickness = 0.0,
        int Index = -1) : origin(Origin), index(Index){
        direction = Direction.normalized();
        thickness = Thickness;
    }
    Ray(const Ray& other)
        :origin(other.origin),
          direction(other.direction),
          index(other.index),
          thickness(other.thickness){}

    inline Ray inverse() const { return Ray(origin, -direction); }

    Ray& operator= (const Ray& other){
        this->origin = other.origin;
        this->direction = other.direction;
        this->index = other.index;
        this->thickness = other.thickness;
        return *this;
    }
};

class HitResult{

public:
    bool hit;
    double distance;

    double u;
    double v;
    int index;

    HitResult(bool isHit = false, double hitDistance = DBL_MAX)
        : hit(isHit), distance(hitDistance)
    {
        u = -1.0;
        v = -1.0;
        index = -1;
    }

    HitResult(const HitResult& other)
    {
        this->hit = other.hit;
        this->distance = other.distance;
        this->u = other.u;
        this->v = other.v;
        this->index = other.index;
    }

    HitResult& operator= (const HitResult& other)
    {
        this->hit = other.hit;
        this->distance = other.distance;
        this->u = other.u;
        this->v = other.v;
        this->index = other.index;

        return *this;
    }
};

#endif // PRIMITIVES_H
