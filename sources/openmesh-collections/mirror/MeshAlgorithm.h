#ifndef MESHALGORITHM_H
#define MESHALGORITHM_H

#include "Mesh.h"

/**
 * @brief mirror
 * @param mesh
 * @return
 */
Mesh mirror(Mesh &mesh);

/**
 * @brief revertFaceNormal
 * 逆置面法向量
 * @param mesh
 */
Mesh revertFaceNormal(Mesh &mesh);

#endif // MESHALGORITHM_H
