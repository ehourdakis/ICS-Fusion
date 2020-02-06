#ifndef KERNELS_CALLS_H
#define KERNELS_CALLS_H
#include"utils.h"

sMatrix6 calculatePoint2PointCov(float3 *vert,
                                 int vertSize,
                                 float3 *prevVert,
                                 int prevVertSize,
                                 int2 *corresp,
                                 int correspSize,
                                 sMatrix4 tf);
#endif
