#include "keyptsmap.h"
#include<fstream>
#define COV 1.0e-5;

keyptsMap::keyptsMap(PoseGraph *isam, IcsFusion *f)
    :_isam(isam),
      _fusion(f)
{
    matcher=cv::FlannBasedMatcher::create();
//    ratio_thresh = 0.7f;
    ratio_thresh = 0.75f;
}

void keyptsMap::clear()
{
    _points.clear();
    _descr.clear();
    lanmarks.clear();
}

void keyptsMap::addKeypoints(std::vector<float3> &keypoints,
                             std::vector<FeatDescriptor> &descriptors)
{
    _points.insert(_points.end(),
                   keypoints.begin(),
                   keypoints.end());
    _descr.insert(_descr.end(),
                   descriptors.begin(),
                   descriptors.end());

    for(uint i=0;i<keypoints.size();i++)
    {
        sMatrix3 cov=descriptors[i].cov;
        //cov=cov*descriptors[i].cov;

        int lidx=_isam->addLandmark(keypoints[i]);
        lanmarks.push_back(lidx);
        _isam->connectLandmark(keypoints[i],lidx,-1,cov);
        
    }
    prevPose=_fusion->getPose();
}

void keyptsMap::getMatches(const std::vector<float3> &keypoints,
                std::vector<float3> &train,
                std::vector<float3> &query)
{
    for(int i=0;i<good_matches.size();i++)
    {
        cv::DMatch m=good_matches[i];
        int tidx=m.trainIdx;
        int qidx=m.queryIdx;

        train.push_back(_points[tidx]);
        query.push_back(keypoints[qidx]);
    }
}

std::vector<cv::DMatch> keyptsMap::goodMatches()
{
    return good_matches;
}


bool keyptsMap::matching(std::vector<float3> &keypoints,
                         std::vector<FeatDescriptor> &descriptors,
                         int frame)
{
    std::vector< std::vector<cv::DMatch> > knn_matches;
    cv::Mat queryDescr=toCvMat(descriptors);
    cv::Mat trainDescr=toCvMat(_descr);

    std::cout<<"queryDescr size:"<<queryDescr.size()<<std::endl;
    std::cout<<"trainDescr size:"<<trainDescr.size()<<std::endl;

    cv::Ptr<cv::FlannBasedMatcher > matcher2=cv::FlannBasedMatcher::create();
    matcher->knnMatch( queryDescr,trainDescr, knn_matches, 2 );


    good_matches.clear();

    std::vector< std::pair<int, int> > matchIdx;
    std::vector<uint> newDescrIdx;

    std::map<int,int> newLanmarks;
    sMatrix4 pose=_fusion->getPose();

    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        cv::DMatch m=knn_matches[i][0];
        int tidx=m.trainIdx;
        int qidx=m.queryIdx;
        float discDist=m.distance;
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {

            float3 p1=prevPose*_points[tidx];
            float3 p2=pose*keypoints[qidx];;

            float dist3d=dist(p1,p2);

            if(discDist<100 )
            {
                if(dist3d<0.5||true )
                {
                    good_matches.push_back(m);
                    sMatrix3 cov=descriptors[qidx].cov;
                    
                    std::pair<int, int> p(tidx, qidx);
                    matchIdx.push_back(p);
//                    std::cout<<cov<<std::endl;
                    int lidx=lanmarks[tidx];
                    _isam->connectLandmark(keypoints[qidx],lidx,-1,cov);
                }
            }
        }
        else if(discDist>150 )
        {
            sMatrix3 cov=descriptors[qidx].cov;
            //cov=cov*1e-7;
//            std::cout<<cov<<std::endl;
           int lidx=_isam->addLandmark(keypoints[qidx]);
           _isam->connectLandmark(keypoints[qidx],lidx,-1,cov);
           _points.push_back(keypoints[qidx]);
           _descr.push_back(descriptors[qidx]);
           lanmarks.push_back(lidx);
           newDescrIdx.push_back(qidx);
        }
    }

    std::cout<<"Feature number "<<keypoints.size()<<std::endl;
    std::cout<<"Matches size "<<good_matches.size()<<std::endl;
    std::cout<<"New Feature number "<<newDescrIdx.size()<<std::endl;

#if 0
    /*Save keypoint map*/
    char buf[32];
    sprintf(buf,"f_/f_%d_map_keypts",frame);
    saveKeypoints(buf,_points);
    sprintf(buf,"f_/f_%d_map_descr",frame);
    saveDescriptors(buf,_descr);


    /*Save new keypoints*/
    sprintf(buf,"f_/f_%d_new_keypts",frame);
    saveKeypoints(buf,keypoints);
    sprintf(buf,"f_/f_%d_new_descr",frame);
    saveDescriptors(buf,descriptors);

    /*Save match keypoints*/
    sprintf(buf,"f_/f_%d_matching",frame);
    saveMatching(buf,matchIdx);
#endif
    //_points=keypoints;
    //_descr=descriptors;

    prevPose=pose;
    return true;
}

void keyptsMap::saveKeypoints(std::string fileName,const std::vector<float3> &keypts)
{
//    std::cout << "Saving keypoints to disk("<<fileName<<")..."<< std::endl;
    std::ofstream keypts_out_file(fileName, std::ios::out);

    for (int i = 0; i < keypts.size(); i++)
    {
        /*
        float f[3];
        f[0]=float(keypts[i].x);
        f[1]=float(keypts[i].y);
        f[2]=float(keypts[i].z);

        keypts_out_file.write((char*)f, 3*sizeof(float));
        */
        keypts_out_file<<keypts[i].x<<" "
                       <<keypts[i].y<<" "
                       <<keypts[i].z<<"\n";
    }
    keypts_out_file.close();
}

void keyptsMap::saveDescriptors(std::string fileName, const std::vector<FeatDescriptor> &desc)
{
//    std::cout << "Saving 3DMatch descriptors to disk (desc.3dmatch.bin)..." << std::endl;
    std::ofstream desc_out_file(fileName, std::ios::binary | std::ios::out);

    for(int i=0;i<desc.size();i++)
    {
        for(uint j=0;j<FeatDescriptor::size();j++)
        {
            desc_out_file<<desc[i].data[j]<<" ";
        }
        desc_out_file<<'\n';
    }
    desc_out_file.close();
}

void keyptsMap::saveMatching(std::string fileName, const std::vector< std::pair<int, int> > &matchIdx)
{
//    std::cout << "Saving 3DMatch descriptors to disk (desc.3dmatch.bin)..." << std::endl;
    std::ofstream out_file(fileName, std::ios::binary | std::ios::out);

    for(int i=0;i<matchIdx.size();i++)
    {
        auto p=matchIdx[i];
        out_file<<p.first<<" "<<p.second<<'\n';

    }
    out_file.close();
}



//void toCvMat(std::vector<FeatDescriptor> &descr,cv::Mat &mat)
//{

//}
