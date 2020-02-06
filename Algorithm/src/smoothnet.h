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
        bool socketConnect();
        void loadFrameData(int frame);
        bool findTf(sMatrix4 &tf,
                    float &fitness,
                    float &rmse,
                    int _frame);

        void saveKeyPts(char *outFileName, int frame);
        void saveKeyVertex(char *outFileName,int frame);

    private:
        bool sendLrfToSoc();
        bool receiveTf(sMatrix4 &mat, float &fitness, float &rmse);
        void sendKeyVertex(int frame);
        void findKeyPts(int frame);
        void calculateLRF(int frame);

        kparams_t _params;
        Image<float3, Host> vertices;
        Image<TrackData, Host> trackData;
        int prevFrame;

        uint2 keypts[1000];
        IcsFusion *_fusion;

        std::vector<int> evaluation_points;
        std::vector<descr_t> descriptors;        

        bool firstTime;
        float3 *keyVert;

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
