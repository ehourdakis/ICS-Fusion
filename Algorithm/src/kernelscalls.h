#ifndef KERNELS_CALLS_H
#define KERNELS_CALLS_H
#include"utils.h"

sMatrix6 calculatePoint2PointCov(const float3 *vert,
                                 int vertSize,
                                 const float3 *prevVert,
                                 int prevVertSize,
                                 const int2 *corresp,
                                 size_t correspSize,
                                 const sMatrix4 &tf);
#endif
