#ifndef FEATURES_DESCRIPTOR_H 
#define FEATURES_DESCRIPTOR_H

#include <opencv2/core/core.hpp>

// #define DESCR_SIZE 128
#define DESCR_SIZE 36
#include"utils.h"
struct FeatDescriptor
{
    float data[DESCR_SIZE];
    float s2;
    sMatrix3 cov;

    //shouldn't declared as int?
    float x;
    float y;

    static uint size()
    {
        return DESCR_SIZE;
    }
};

inline cv::Mat toCvMat(const std::vector<FeatDescriptor> &descr)
{
    int rows=descr.size();
    int col=DESCR_SIZE;
    cv::Mat mat(rows,col,CV_32F);

    for(int i=0;i<rows;i++)
    {
        for(int j=0;j<col;j++)
        {
            const FeatDescriptor &d=descr[i];
            mat.at<float>(i,j)=d.data[j];
        }
    }
    return mat;
}

inline void fromCvMat(std::vector<FeatDescriptor> &descr,cv::Mat &mat)
{
    int rows=mat.rows;
    descr.clear();
    descr.reserve(rows);
    for(int j=0; j<rows; j++)
    {
        FeatDescriptor d;
        for(int i=0;i<DESCR_SIZE;i++)
        {
            d.data[i]=mat.at<float>(j,i);
        }
        descr.push_back(d);
    }
}

inline void fromCvMatRow(FeatDescriptor &descr,cv::Mat &mat,uint row)
{
    for(int i=0;i<DESCR_SIZE;i++)
    {
        descr.data[i]=mat.at<float>(row,i);
    }
}

#endif
