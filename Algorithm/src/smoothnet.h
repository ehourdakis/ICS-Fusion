#ifndef SMOOTH_NET_H 
#define SMOOTH_NET_H

#include"utils.h"
#include"icsFusion.h"
#include<kparams.h>
#include <Eigen/Dense>
class SmoothNet
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        struct descr_t
        {
            float data[32];
        };

        SmoothNet(IcsFusion *f,const kparams_t &params);
        ~SmoothNet();

        void clear();
        bool socketConnect();
        void loadFrameData(int frame);
        bool findDescriptors(int frame);

        /*
        bool getTf(sMatrix4 &tf,
                    float &fitness,
                    float &rmse,
                    sMatrix6 &cov,
                    int _frame);
        */

        sMatrix4 getTf() const
        {
            return last_tf;
        }

        float getFitness() const
        {
            return last_fitness;
        }

        float getRmse() const
        {
            return last_rmse;
        }

        sMatrix6 calculateCov();

        void saveKeyPts(char *outFileName, int frame);
        void saveKeyVertex(char *outFileName,int frame);


    private:
        bool sendLrfToSoc();
        bool receiveTf(sMatrix4 &mat, float &fitness, float &rmse);
        bool receiveCorresp();
        void sendKeyVertex(int frame);
        bool findKeyPts(int frame);
        void calculateLRF(int frame);

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
