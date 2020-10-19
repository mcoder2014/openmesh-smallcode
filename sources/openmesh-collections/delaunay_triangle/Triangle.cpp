#include "Triangle.h"

#include <string>

using namespace std;

Triangle::Triangle(Vector3D* v0, Vector3D* v1, Vector3D* v2)
{
    Id = generateRunningId();
    Vertex[0] = v0;
    Vertex[1] = v1;
    Vertex[2] = v2;
}

Triangle::~Triangle()
{
}

int Triangle::generateRunningId()
{
    static int id = 0;
    return id++;
}

bool Triangle::hasVertexCoincidentWith(Vector3D* dot)
{
    return Vertex[0]->isCoincidentWith(dot)
        || Vertex[1]->isCoincidentWith(dot)
        || Vertex[2]->isCoincidentWith(dot);
}

void Triangle::assignNeighbors(Triangle* n0, Triangle* n1, Triangle* n2)
{
    Neighbor[0] = n0;
    Neighbor[1] = n1;
    Neighbor[2] = n2;
}

string Triangle::toString()
{
    return "Triangle ID: " + to_string(Id) + ";\n"
        + "Vertex[0]: " + Vertex[0]->toString()
        + "Vertex[1]: " + Vertex[1]->toString()
        + "Vertex[2]: " + Vertex[2]->toString()
        + "Neighbor[0] ID: " + to_string(Neighbor[0]->Id) + ", "
        + "Neighbor[1] ID: " + to_string(Neighbor[1]->Id) + ", "
        + "Neighbor[2] ID: " + to_string(Neighbor[2]->Id) + ";\n";
}
