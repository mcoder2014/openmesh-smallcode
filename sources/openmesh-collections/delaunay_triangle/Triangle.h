#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "Vector3D.h"

class Triangle
{
private:
    int generateRunningId();
public:
    int Id = 0;

    // pointers pointing to 3 vertices
    Vector3D* Vertex[3];

    // pointers pointing to 3 neighbors
    Triangle* Neighbor[3];

    Triangle(Vector3D* v0, Vector3D* v1, Vector3D* v2);
    ~Triangle();

    bool hasVertexCoincidentWith(Vector3D* dot);
    void assignNeighbors(Triangle* n0, Triangle* n1, Triangle* n2);
    std::string toString();
};



#endif // TRIANGLE_H
