#ifndef MATCH_3D_H
#define MATCH_3D_H

#include"volume.h"

//3dMatch headers
#include <cnet.h>
#include"kparams.h"

class Match3D
{
    public:
        Match3D(kparams_t p);
        ~Match3D();

        void findDescriptors(Volume vol, int num, uint3 *kp,
                             float *desc, char *empty);

        void createTempKeyPoints(uint3 *kp,uint3 size,uint num);
        void createKeyPoints(uint3 *kp,uint3 size,uint num);

        //Is this value fixed?
        inline int descSize() const
        {
            return desc_size;
        }

        void saveKeypoints(std::string fileName, uint3 *keypts,
                           char *isEmpty, int num_keypts);
        void saveDescriptors(std::string fileName,float *desc,
                             char *isEmpty,int num_keypts);
    private:
        int num_keypts;
        marvin::CNet net;
        marvin::Response * rData;
        marvin::Response * rFeat;
        kparams_t params;
        uint3 *keyptsTmp;
        float *descTmp;

        int batch_size;
        int desc_size;        
};
#endif
