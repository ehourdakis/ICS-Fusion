#ifndef SMOOTH_NET_H 
#define SMOOTH_NET_H

#include"utils.h"
#include"icsFusion.h"
class SmoothNet
{
    public:
        SmoothNet(IcsFusion *f);
        
        void calculateLRF();
        void readKeyPts(const Image<float3, Host> &vert);

        void calculatePointLRF(const uint2 pt);
        void calculateLRFPtr(uint2 pt);
    private:

        bool calculateLRFHost(uint2 pt, sMatrix3 &covar);

        uint2 keypts[1000];
        IcsFusion *_fusion;

        std::vector<int> evaluation_points;


        float radius;
        int num_voxels;
        float smoothing_kernel_width;
        float voxel_step_size;
        float grid_size;
        float lrf_radius;
        float smoothing_factor;

};
#endif
