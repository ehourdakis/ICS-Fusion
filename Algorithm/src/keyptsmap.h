#ifndef KEYPTSMAP_H
#define KEYPTSMAP_H

#include"featursDescriptor.h"
#include"PoseGraph.h"
#include"icsFusion.h"
//=====OpenCv=====
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <map>
class keyptsMap
{
    public:
        keyptsMap(PoseGraph *isam,IcsFusion *f);
        void addKeypoints(std::vector<float3> &keypoints,
                           std::vector<FeatDescriptor> &descriptors);

        bool matching(std::vector<float3> &keypoints,
                      std::vector<FeatDescriptor> &descriptors, int frame);
        void clear();

        bool isEmpty()
        {
            return _points.size()==0;
        }

    private:

//        void toCvMat(std::vector<FeatDescriptor> &descr,
//                     cv::Mat &mat);

        //std::map<int,int> lanmarks;
        std::vector<int> lanmarks;
        cv::Ptr<cv::FlannBasedMatcher> matcher;
        PoseGraph *_isam;
        IcsFusion *_fusion;

        float ratio_thresh;

        std::vector<float3> _points;
        std::vector<FeatDescriptor> _descr;
        sMatrix4 prevPose;

        void saveDescriptors(std::string fileName, const std::vector<FeatDescriptor> &desc);
        void saveKeypoints(std::string fileName,const std::vector<float3> &keypts);
        void saveMatching(std::string fileName, const std::vector< std::pair<int, int> > &matchIdx);
        //void saveDescData(const std::vector<float3> &keypts, const std::vector<FeatDescriptor> &desc, int _frame);
};

#endif // KEYPTSMAP_H
