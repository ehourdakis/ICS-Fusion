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
#include"kparams.h"

//Open3d
#include<Open3D/Registration/Feature.h>

class keyptsMap
{
    public:
        keyptsMap(PoseGraph *isam,IcsFusion *f,const kparams_t &p);
        void addKeypoints(std::vector<float3> &keypoints,
                           std::vector<FeatDescriptor> &descriptors,int frame);

        bool matching(std::vector<float3> &keypoints,
                      std::vector<FeatDescriptor> &descriptors, int frame);
        void clear();

        bool isEmpty()
        {
            return eigenPts.size()==0;
        }

        void getMatches(const std::vector<float3> &keypoints,
                        std::vector<float3> &train,
                        std::vector<float3> &query);

        std::vector<cv::DMatch> goodMatches();
        void saveMap(char *descrFile,char *poitsFile,char *frameFile);
    private:
        int prevFrame;
        const kparams_t &params;
        double max_correspondence_distance;

        std::vector<Eigen::Vector3d> eigenPts;
        std::vector<Eigen::Vector3d> prevEigenPts;

        open3d::registration::Feature *descr;
        open3d::registration::Feature *prevDescr;


        std::vector<int> lanmarks;
        cv::Ptr<cv::FlannBasedMatcher> matcher;
        PoseGraph *_isam;
        IcsFusion *_fusion;

        float ratio_thresh;

        std::vector<float3> _points;
        std::vector<FeatDescriptor> _descr;
        sMatrix4 prevPose;

        std::vector<cv::DMatch> good_matches;
        std::vector<uint2> descrFrame;

        void saveDescriptors(std::string fileName, const std::vector<FeatDescriptor> &desc);
        void saveKeypoints(std::string fileName,const std::vector<float3> &keypts);
        void saveMatching(std::string fileName, const std::vector< std::pair<int, int> > &matchIdx);
        //void saveDescData(const std::vector<float3> &keypts, const std::vector<FeatDescriptor> &desc, int _frame);
};

#endif // KEYPTSMAP_H
