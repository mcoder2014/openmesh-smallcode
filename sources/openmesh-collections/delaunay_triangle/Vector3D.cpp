#include "Vector3D.h"

#include <cmath>
#include <string>

using namespace std;

Vector3D::Vector3D(double x, double y, double z, uint8_t r, uint8_t g, uint8_t b)
{
    Id = generateRunningId();

    X = x;
    Y = y;
    Z = z;

    R = r;
    G = g;
    B = b;
}

Vector3D::Vector3D(double x, double y, double z, bool isAuxiliaryDot, uint8_t r, uint8_t g, uint8_t b)
{
    Id = generateRunningId();

    IsAuxiliaryDot = isAuxiliaryDot;

    X = x;
    Y = y;
    Z = z;

    R = r;
    G = g;
    B = b;
}

Vector3D::Vector3D(Vector3D* dot, double lengthAfterProjection)
{
    Id = dot->Id;
    IsVisited = dot->IsVisited;
    IsAuxiliaryDot = dot->IsAuxiliaryDot;

    double length = sqrt(pow(dot->X, 2) + pow(dot->Y, 2) + pow(dot->Z, 2));
    double scaleFactor = lengthAfterProjection / length;

    X = scaleFactor * dot->X;
    Y = scaleFactor * dot->Y;
    Z = scaleFactor * dot->Z;

    R = dot->R;
    G = dot->G;
    B = dot->B;
}

Vector3D::~Vector3D()
{
}

int Vector3D::generateRunningId()
{
    static int id = 0;
    return id++;
}

bool Vector3D::isCoincidentWith(Vector3D* dot)
{
    return (X == dot->X && Y == dot->Y && Z == dot->Z);
}

string Vector3D::toString()
{
    return "Dot ID: " + to_string(Id) + "; "
        + to_string(X) + ", " + to_string(Y) + ", " + to_string(Z) + "; "
        + to_string(R) + ", " + to_string(G) + ", " + to_string(B) + "; "
        + "IsVisited: " + (IsVisited ? "true" : "false") + "; "
        + "IsAuxiliaryDot: " + (IsAuxiliaryDot ? "true" : "false") + ";\n";
}
