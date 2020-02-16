#ifndef SMOOTH_NET_H 
#define SMOOTH_NET_H

#include"utils.h"
#include"icsFusion.h"
#include<kparams.h>
#include <Eigen/Dense>


class SmoothNet
{
    public:
        SmoothNet(int num_voxels,);
        
        void calculateLRF();
    private:
        int num_voxels
        
        
        const kparams_t &params;
        Image<float3, Host> vertices;
        Image<TrackData, Host> trackData;
        int prevFrame;

        float last_rmse;
        float last_fitness;
        sMatrix4 last_tf;

        uint2 keypts[1000];
        IcsFusion *_fusion;

        std::vector<int> evaluation_points;
        std::vector<descr_t> descriptors;        

        bool firstTime;
        float3 *keyVert;
        float3 *prevKeyVert;

        float3 *vertBuff1;
        float3 *vertBuff2;

        int2 *corresp;
        int correspSize;

        float radius;
        int num_voxels;
        int counter_voxel;
        float smoothing_kernel_width;
        float voxel_step_size;
        float grid_size;
        float lrf_radius;
        float smoothing_factor;

        float **lrf;

        int sock;

};
#endif
