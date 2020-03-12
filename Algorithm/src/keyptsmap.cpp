#include "keyptsmap.h"
#include<fstream>

#include<Open3D/Registration/Registration.h>
#include<Open3D/Geometry/PointCloud.h>
#include<Open3D/Registration/CorrespondenceChecker.h>
#include"kernelscalls.h"
//#define COV 1.0e-5;


keyptsMap::keyptsMap(PoseGraph *isam, IcsFusion *f,const kparams_t &p)
    :_isam(isam),
      _fusion(f),
      params(p)
{
    descr=new open3d::registration::Feature();
//    mapDescr=new open3d::registration::Feature();

    max_correspondence_distance=0.1;
    //max_correspondence_distance=10;
}

void keyptsMap::clear()
{
    _points.clear();
    _descr.clear();
    lanmarks.clear();
    descrFrame.clear();

    eigenPts.clear();
    prevEigenPts.clear();
}

void keyptsMap::addKeypoints(std::vector<float3> &keypoints,
                             std::vector<FeatDescriptor> &descriptors,
                             int frame)
{
    _descr.insert(_descr.end(),
                   descriptors.begin(),
                   descriptors.end());     


    for(int i=0;i<keypoints.size();i++)
    {

        sMatrix3 cov=descriptors[i].cov;
        int lidx=_isam->addLandmark(keypoints[i]);
        _isam->connectLandmark(keypoints[i],lidx,frame,cov);
        lanmarks.push_back(lidx);
    }
    prevFrame=frame;


    std::cout<<"Added:"<<descriptors.size()<<" keypts."<<std::endl;
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
    using namespace open3d::geometry;
    using namespace open3d::registration;
    good_matches.clear();

    if(_descr.size()==0 || descriptors.size()==0)
        return false;

    std::vector<Eigen::Vector3d> map3d;
    descr->Resize(DESCR_SIZE,_descr.size());

    for(int i=0;i<_isam->landmarksSize();i++)
    {
        float3 lpos=_isam->landmarkPos(i);

        Eigen::Vector3d v(lpos.x,
                          lpos.y,
                          lpos.z);

        map3d.push_back(v);
        for(int j=0;j<DESCR_SIZE;j++)
        {
            descr->data_(j,i)=_descr[i].data[j];
        }
    }

    std::vector<Eigen::Vector3d> newPts;
    newPts.reserve(descriptors.size());
    Feature *newDescr=new Feature();
    newDescr->Resize(DESCR_SIZE,descriptors.size());
    for(int i=0;i<keypoints.size();i++)
    {

        Eigen::Vector3d v(keypoints[i].x,
                          keypoints[i].y,
                          keypoints[i].z);

        newPts.push_back(v);

        for(int j=0;j<DESCR_SIZE;j++)
        {
            newDescr->data_(j,i)=descriptors[i].data[j];
        }
    }

    PointCloud cloud0=PointCloud(map3d);
    PointCloud cloud1=PointCloud(newPts);


    std::vector<std::reference_wrapper<const CorrespondenceChecker>>correspondence_checker;
    auto correspondence_checker_edge_length =CorrespondenceCheckerBasedOnEdgeLength(0.9);
    auto correspondence_checker_distance =
        CorrespondenceCheckerBasedOnDistance(max_correspondence_distance);

    correspondence_checker.push_back(correspondence_checker_edge_length);
    correspondence_checker.push_back(correspondence_checker_distance);

    const RANSACConvergenceCriteria criteria =RANSACConvergenceCriteria(4000000, 500);



    RegistrationResult results=RegistrationRANSACBasedOnFeatureMatching(
                 cloud1,cloud0,*descr,*newDescr,max_correspondence_distance,
                 TransformationEstimationPointToPoint(false),
                 4,//ransac_n
                 correspondence_checker,
                 criteria );

    CorrespondenceSet corr=results.correspondence_set_;

    char *hasCorr=new char[keypoints.size()];
    memset(hasCorr,0,keypoints.size());


    for(int i=0;i<corr.size();i++)
    {
        Eigen::Vector2i c=corr[i];
        int idx1=c(1);
        int idx2=c(0);

        FeatDescriptor &d2=descriptors[idx2];
        float3 p2=keypoints[idx2];
        sMatrix3 cov2=d2.cov;
        _isam->connectLandmark(p2,idx1,-1,cov2);
        hasCorr[idx2]=1;
    }

    for(int i=0;i<keypoints.size();i++)
    {
        if(hasCorr[i]==0)
        {
            FeatDescriptor &d=descriptors[i];
            int lid=_isam->addLandmark(keypoints[i]);
            _isam->connectLandmark(keypoints[i],lid,-1,d.cov);
            lanmarks.push_back(lid);

            _descr.push_back(d);
        }
    }

    delete newDescr;
    delete hasCorr;

    std::cout<<"Found:"<<keypoints.size()<<" keypts;"<<std::endl;
    std::cout<<"Found:"<<corr.size()<<" matches;"<<std::endl;
    return true;










#if 0
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
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance )
        {

            float3 p1=prevPose*_points[tidx];
            float3 p2=pose*keypoints[qidx];;

            float dist3d=dist(p1,p2);

            if(discDist<10000 )
            {
                if(dist3d<0.5||true )
                {
                    good_matches.push_back(m);
                    sMatrix3 cov=descriptors[qidx].cov;
                    
                    std::pair<int, int> p(tidx, qidx);
                    matchIdx.push_back(p);
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
           
           descrFrame.push_back(make_uint2(frame,qidx) );
        }
    }

    std::cout<<"Feature number "<<keypoints.size()<<std::endl;
    std::cout<<"Matches size "<<good_matches.size()<<std::endl;
    std::cout<<"New Feature number "<<newDescrIdx.size()<<std::endl;

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

//    prevPose=pose;
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
    std::ofstream desc_out_file(fileName, std::ios::out);

    for(int i=0;i<desc.size();i++)
    {
        for(uint j=0;j<FeatDescriptor::size();j++)
        {
            desc_out_file<<desc[i].data[j]<<" ";
        }
        desc_out_file<<desc[i].s2<<'\n';
    }
    desc_out_file.close();
}

void keyptsMap::saveMap(char *descrFile,char *poitsFile,char *frameFile)
{
    saveDescriptors( std::string(descrFile),_descr );
    saveKeypoints( std::string(poitsFile),_points );
    
    std::ofstream outFile(frameFile, std::ios::out);
    for(int i=0;i<descrFrame.size();i++)
    {
        uint2 p=descrFrame[i];
        outFile<<p.x<<" "<<p.y<<"\n";
    }
    outFile.close();
}



//void toCvMat(std::vector<FeatDescriptor> &descr,cv::Mat &mat)
//{

//}
