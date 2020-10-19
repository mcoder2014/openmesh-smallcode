#ifndef VECTOR3D_H
#define VECTOR3D_H

#include <iostream>

class Vector3D
{
private:
    int generateRunningId();
public:
    int Id = 0;

    // coordinate
    double X, Y, Z;

    // color
    uint8_t R, G, B;

    bool IsVisited = false;
    bool IsAuxiliaryDot = false;

    Vector3D(double x, double y, double z, uint8_t r = 255, uint8_t g = 248, uint8_t b = 220);
    Vector3D(double x, double y, double z, bool isAuxiliaryDot, uint8_t r = 255, uint8_t g = 248, uint8_t b = 220);
    Vector3D(Vector3D* dot, double lengthAfterProjection);
    ~Vector3D();

    bool isCoincidentWith(Vector3D* dot);
    std::string toString();
};
#endif // VECTOR3D_H
