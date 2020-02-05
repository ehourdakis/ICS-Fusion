#ifndef SMOOTH_NET_H 
#define SMOOTH_NET_H

#include"utils.h"
#include"icsFusion.h"
#include<kparams.h>

class SmoothNet
{
    public:

        struct descr_t
        {
            float data[32];
        };

        SmoothNet(IcsFusion *f,kparams_t params);
        ~SmoothNet();
        void clear();
                
        void loadFrameData(int frame);
        void findKeyPts(int frame);
        void calculateLRF(int frame);
        bool callCnn(int frame);
        bool readDescriptorCsv();
        void readKeyPts();
        void saveKeyPts(int frame);
        void saveKeyVertex(int frame);

        float findTransformation(sMatrix4 &mat, float &rmse, int frame);

    private:
        kparams_t _params;
        Image<float3, Host> vertices;
        Image<TrackData, Host> trackData;
        int prevFrame;

        uint2 keypts[1000];
        IcsFusion *_fusion;

        std::vector<int> evaluation_points;
        std::vector<descr_t> descriptors;        

        bool firstTime;
        char sdv_file[256];
        char descr_file[256];
        char prev_descr_file[256];
        char key_vert_file[256];
        char prev_key_vert_file[256];
        char trans_file[256];

        float radius;
        int num_voxels;
        float smoothing_kernel_width;
        float voxel_step_size;
        float grid_size;
        float lrf_radius;
        float smoothing_factor;

};
#endif
