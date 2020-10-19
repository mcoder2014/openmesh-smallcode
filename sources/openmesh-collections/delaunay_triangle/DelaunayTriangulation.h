#ifndef DELAUNAYTRIANGULATION_H
#define DELAUNAYTRIANGULATION_H

#define INIT_VERTICES_COUNT 6 /* count of vertices in the initial hull */
#define INIT_FACES_COUNT 8 /* count of faces in the initial hull */
#define VECTOR_LENGTH 1 /* radius of unit sphere the dots projected into */

#include <vector>

#include "Vector3D.h"
#include "Triangle.h"

using namespace std;

class DelaunayTriangulation
{
private:
    Vector3D * _AuxiliaryDots[INIT_VERTICES_COUNT];
    std::vector<Vector3D*>* _ProjectedDots;
    std::vector<Triangle*>* _Mesh;

    // 0: triangle search operations
    // 1: local optimizations
    // 2: start time; 3: end time;
    long _Statistics[4];

    void buildInitialHull(std::vector<Vector3D*>* dots);
    void insertDot(Vector3D* dot);
    void removeExtraTriangles();
    void splitTriangle(Triangle* triangle, Vector3D* dot);
    void fixNeighborhood(Triangle* target, Triangle* oldNeighbor, Triangle* newNeighbor);
    void localOptimization(Triangle* t0, Triangle* t1);
    bool SwapDiagonal(Triangle* t0, Triangle* t1);
    bool isMinimumValueInArray(double arr[], int length, int index);
    double getDistance(Vector3D* v0, Vector3D* v1);
    double getDeterminant(Vector3D* v0, Vector3D* v1, Vector3D* v2);
    double getDeterminant(double matrix[]);

public:
    DelaunayTriangulation();
    ~DelaunayTriangulation();

    std::vector<std::tuple<int, int, int>*> getTriangulationResult(std::vector<Vector3D*> &dots);
    std::string getStatistics();
};
#endif // DELAUNAYTRIANGULATION_H
