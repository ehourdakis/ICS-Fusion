#ifndef IMAGEPROCESS_H
#define IMAGEPROCESS_H

#include <ros/ros.h>
#include <kernels.h>

class ImageProcess
{
    public:
        ImageProcess(uint2 size)
            :computationSize(size){}

        void setImages(const uchar3 *cam_image,const uchar3 *voxel_image);
    private:
        uint2 computationSize;
};

#endif // IMAGEPROCESS_H
