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
    prevDescr=new open3d::registration::Feature();

   max_correspondence_distance=0.05;
//    max_correspondence_distance=10;
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
     _points.insert(_points.end(),
                   keypoints.begin(),
                   keypoints.end());


    descr->Resize(DESCR_SIZE,descriptors.size());    

    for(int i=0;i<keypoints.size();i++)
    {
         Eigen::Vector3d v(keypoints[i].x,
                           keypoints[i].y,
                           keypoints[i].z);

//        Eigen::Vector3d v(descriptors[i].x,
//                           descriptors[i].y,
//                           0);

        eigenPts.push_back(v);

        for(int j=0;j<DESCR_SIZE;j++)
        {
            descr->data_(j,i)=descriptors[i].data[j];
        }
    }
    prevFrame=frame;
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

    std::swap(prevEigenPts,eigenPts);
    std::swap(prevDescr,descr);

    eigenPts.clear();
    eigenPts.reserve(keypoints.size());

    descr->Resize(DESCR_SIZE,descriptors.size());

    for(int i=0;i<keypoints.size();i++)
    {

        Eigen::Vector3d v(keypoints[i].x,
                           keypoints[i].y,
                           keypoints[i].z);


//       Eigen::Vector3d v(descriptors[i].x,
//                         descriptors[i].y,
//                         0);

        eigenPts.push_back(v);

        for(int j=0;j<DESCR_SIZE;j++)
        {
            descr->data_(j,i)=descriptors[i].data[j];
        }
    }
    
    std::vector<int> prev_corr;
    std::vector<int> next_corr; 

    if(prevEigenPts.size()>0 && eigenPts.size()>0)
    {


        PointCloud cloud0=PointCloud(prevEigenPts);
        PointCloud cloud1=PointCloud(eigenPts);


        std::vector<std::reference_wrapper<const CorrespondenceChecker>>
            correspondence_checker;
        auto correspondence_checker_edge_length =
            CorrespondenceCheckerBasedOnEdgeLength(0.9);
        auto correspondence_checker_distance =
            CorrespondenceCheckerBasedOnDistance(max_correspondence_distance);

        correspondence_checker.push_back(correspondence_checker_edge_length);
        correspondence_checker.push_back(correspondence_checker_distance);

        const RANSACConvergenceCriteria criteria =RANSACConvergenceCriteria(4000000, 500);


        RegistrationResult results=RegistrationRANSACBasedOnFeatureMatching(
                     cloud1,cloud0,*descr,*prevDescr,max_correspondence_distance,
                     TransformationEstimationPointToPoint(false),
                     4,//ransac_n
                     correspondence_checker,
                     criteria );

//        RegistrationResult results=RegistrationRANSACBasedOnFeatureMatching(
//                            cloud1,cloud0,*descr,*prevDescr,max_correspondence_distance);

        sMatrix4 tf;
        for(int i=0;i<4;i++)
        {
            for(int j=0;j<4;j++)
                tf(i,j)=results.transformation_(i,j);
        }

        //std::cout<<tf<<std::endl;

        CorrespondenceSet corr=results.correspondence_set_;

        _isam->clearLandmarks();

        std::cout<<"Ransac fitness:"<<results.fitness_<<std::endl;
        std::cout<<"Ransac rmse:"<<results.inlier_rmse_<<std::endl;

        if(corr.size()==0)
            return false;
        if(results.fitness_<0.7)
            return false;

        
        for(int i=0;i<corr.size();i++)
        {
            Eigen::Vector2i c=corr[i];
            int idx1=c(1);
            int idx2=c(0);

            cv::DMatch m( idx2,idx1,1 );
            good_matches.push_back(m);

            FeatDescriptor d1=_descr[idx1];
            FeatDescriptor d2=descriptors[idx2];

            float3 p1=_points[idx1];
            float3 p2=keypoints[idx2];


            sMatrix3 cov1=d1.cov;
            sMatrix3 cov2=d2.cov;
            
            prev_corr.push_back(idx1);
            next_corr.push_back(idx2);
            
//            int lidx=_isam->addLandmark(p1);
//            _isam->connectLandmark(p1,lidx,prevFrame,cov1);
//            _isam->connectLandmark(p2,lidx,-1,cov2);
        }        

//        std::cout<<tf<<std::endl;
        
        
            sMatrix6 cov=calculatePoint2PointCov(keypoints,
                                         keypoints.size(),
                                         _points,
                                         _points.size(),
                                         next_corr,
                                         prev_corr,
                                         tf,
                                         params);
//         cov=cov*0.00001;
        std::cout<<cov<<std::endl;
        
         _isam->addPoseConstrain(prevFrame,-1,tf,cov);
    }


    return true;
//    geometry::PointCloud cloud1=PointCloud();

//    RegistrationResult results=RegistrationRANSACBasedOnFeatureMatching();













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
