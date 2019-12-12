#ifndef KPARAMS_H
#define KPARAMS_H

#include <vector_types.h>
#include"cutil_math.h"
#include<vector>

typedef struct
{
    int compute_size_ratio=1;
    int integration_rate=1;
    int rendering_rate = 1;
    int tracking_rate=1;
    uint3 volume_resolution = make_uint3(256,256,256);
//     uint3 volume_resolution = make_uint3(512,512,512);
    float3 volume_direction = make_float3(4,4,4);
    float3 volume_size = make_float3(8,8,8);

    std::vector<int> pyramid = {10,5,4};
    float mu = 0.1;
    float icp_threshold = 1e-5;

    uint2 inputSize;
    uint2 computationSize;
    float4 camera;

} kparams_t;


#endif // KPARAMS_H
