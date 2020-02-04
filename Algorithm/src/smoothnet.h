#ifndef SMOOTH_NET_H 
#define SMOOTH_NET_H

#include"utils.h"
#include"icsFusion.h"
class SmoothNet
{
    public:

        struct descr_t
        {
            float data[32];
        };

        SmoothNet(IcsFusion *f);
        
        void calculateLRF(int frame);
        void readKeyPts();

        void calculatePointLRF(const uint2 pt);
        void calculateLRFPtr(uint2 pt);
        bool callCnn(int frame);
        bool readDescriptorCsv();
    private:

        bool calculateLRFHost(uint2 pt, sMatrix3 &covar);

        uint2 keypts[1000];
        IcsFusion *_fusion;

        std::vector<int> evaluation_points;
        std::vector<descr_t> descriptors;

        char sdv_file[256];
        char descr_file[256];

        float radius;
        int num_voxels;
        float smoothing_kernel_width;
        float voxel_step_size;
        float grid_size;
        float lrf_radius;
        float smoothing_factor;

};
#endif
