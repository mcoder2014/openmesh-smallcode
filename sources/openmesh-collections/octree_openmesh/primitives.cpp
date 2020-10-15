#include "primitives.h"


int planeBoxOverlap(const Eigen::Vector3d &normal, const Eigen::Vector3d &vert, const Eigen::Vector3d &maxbox)
{
    Eigen::Vector3d vmin,vmax;
    for(int q = 0; q <= 2; q++){
        double v = vert[q];
        if(normal[q] > 0.0){
            vmin[q]=-maxbox[q] - v;
            vmax[q]= maxbox[q] - v;
        }
        else{
            vmin[q]= maxbox[q] - v;
            vmax[q]=-maxbox[q] - v;
        }
    }
    if(normal.dot(vmin) > 0.0) return 0;
    if(normal.dot(vmax) >= 0.0) return 1;

    return 0;
}


