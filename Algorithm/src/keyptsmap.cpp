#include "keyptsmap.h"
#include<fstream>

#include<Open3D/Registration/Registration.h>
#include<Open3D/Geometry/PointCloud.h>
#include<Open3D/Registration/CorrespondenceChecker.h>
#include"kernelscalls.h"
//#define COV 1.0e-5;

#include<teaser/registration.h>

keyptsMap::keyptsMap(PoseGraph *isam, IcsFusion *f,const kparams_t &p)
    :_isam(isam),
      _fusion(f),
      params(p)
{
    descr=new open3d::registration::Feature();
    prevDescr=new open3d::registration::Feature();

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
    _points.insert(_points.end(),
                   keypoints.begin(),
                   keypoints.end());


    descr->Resize(DESCR_SIZE,descriptors.size());    

    for(int i=0;i<keypoints.size();i++)
    {
#if 1
         Eigen::Vector3d v(keypoints[i].x,
                           keypoints[i].y,
                           keypoints[i].z);
#else
        Eigen::Vector3d v(descriptors[i].x,
                          descriptors[i].y,
                          0);
#endif
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

void keyptsMap::teaser(std::vector<FeatDescriptor> &descriptors, sMatrix4 &tf)
{
    std::vector< std::vector<cv::DMatch> > knn_matches;
    cv::Mat queryDescr=toCvMat(descriptors);
    cv::Mat trainDescr=toCvMat(_descr);

    cv::Ptr<cv::FlannBasedMatcher > matcher2=cv::FlannBasedMatcher::create();
    matcher2->knnMatch( queryDescr,trainDescr, knn_matches, 1 );

    good_matches.clear();
    Eigen::Matrix<double, 3, Eigen::Dynamic>src(3,knn_matches.size());
    Eigen::Matrix<double, 3, Eigen::Dynamic>dst(3,knn_matches.size());

    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        cv::DMatch m=knn_matches[i][0];
        int qidx=m.queryIdx;
        int tidx=m.trainIdx;

        src.col(i)<<prevEigenPts[tidx];
        dst.col(i)<<eigenPts[qidx];
    }


    teaser::RobustRegistrationSolver::Params tparams;
    tparams.noise_bound =  0.05;
    tparams.cbar2 = 1;
    tparams.estimate_scaling = false;
    tparams.rotation_max_iterations = 100;
    tparams.rotation_gnc_factor = 1.4;
    tparams.rotation_estimation_algorithm =
        teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    tparams.rotation_cost_threshold = 0.005;

    // Solve with TEASER++
    teaser::RobustRegistrationSolver solver(tparams);

    solver.solve(src, dst);
    auto solution = solver.getSolution();
    std::cout << "Teaser rot:" << std::endl;

    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            tf(i,j)=solution.rotation(i,j);
        }
    }
    tf(0,3)=solution.translation(0);
    tf(1,3)=solution.translation(1);
    tf(2,3)=solution.translation(2);
    //std::cout << solution.rotation << std::endl;
    //std::cout << solution.translation << std::endl;

    tf=inverse(tf);
    std::cout << tf << std::endl;


    if(!solution.valid)
    {
        std::cout<<"Not valid solution"<<std::endl;
        return;
    }



    std::vector<int> inliners=solver.getTranslationInliers();
    float fitness=(float)inliners.size()/(float)knn_matches.size();
    std::cout << "Fitness:"<<fitness << std::endl;
    std::cout << "Inliners size:"<< inliners.size()<<std::endl;
    if(fitness<0.1)
    {
        return;
    }
    if( inliners.size()<10)
    {
        std::cout<<"Too few inliners."<<std::endl;
        return;
    }


    for(int i=0;i<inliners.size();i++)
    {
        int inidx=inliners[i];
        cv::DMatch m=knn_matches[inidx][0];
        good_matches.push_back(m);
    }


}

void keyptsMap::ransac(std::vector<FeatDescriptor> &descriptors,sMatrix4 &tf)
{
    using namespace open3d::geometry;
    using namespace open3d::registration;

    good_matches.clear();
    
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
                 cloud1,cloud0,
                 *descr,*prevDescr,
                 //*prevDescr,*descr,
                 max_correspondence_distance,
                 TransformationEstimationPointToPoint(false),
                 4,//ransac_n
                 correspondence_checker,
                 criteria );

//        RegistrationResult results=RegistrationRANSACBasedOnFeatureMatching(
//                            cloud1,cloud0,*descr,*prevDescr,max_correspondence_distance);

    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
            tf(i,j)=results.transformation_(i,j);
    }

    std::cout<<tf<<std::endl;

//    tf=inverse(tf);
     std::cout<<inverse(tf)<<std::endl;

    std::cout<<"Fitness:"<<results.fitness_<<std::endl;
    std::cout<<"RMSE:"<<results.inlier_rmse_<<std::endl;

    if(results.fitness_ <params.rfitness ||
       results.inlier_rmse_ >params.rerror )
    {
        return;
    }
    
    CorrespondenceSet corr=results.correspondence_set_;

    for(int i=0;i<corr.size();i++)
    {
        Eigen::Vector2i c=corr[i];
        int idx1=c(1);
        int idx2=c(0);

        cv::DMatch m( idx2,idx1,1 );
        good_matches.push_back(m);
    }


}

bool keyptsMap::matching(std::vector<float3> &keypoints,
                         std::vector<FeatDescriptor> &descriptors,
                         int frame)
{
    good_matches.clear();

    std::swap(prevEigenPts,eigenPts);
    std::swap(prevDescr,descr);

//    std::swap(cloud1,cloud2);

    eigenPts.clear();
    eigenPts.reserve(keypoints.size());

    descr->Resize(DESCR_SIZE,descriptors.size());

    for(int i=0;i<keypoints.size();i++)
    {
#if 1
        Eigen::Vector3d v(keypoints[i].x,
                          keypoints[i].y,
                          keypoints[i].z);
#else
       Eigen::Vector3d v(descriptors[i].x,
                         descriptors[i].y,
                         0);
#endif
        eigenPts.push_back(v);

        for(int j=0;j<DESCR_SIZE;j++)
        {
            descr->data_(j,i)=descriptors[i].data[j];
        }

        cloud2.push_back({keypoints[i].x,
                          keypoints[i].y,
                          keypoints[i].z,});
    }
    
    std::vector<int> prev_corr;
    std::vector<int> next_corr; 

    if(prevEigenPts.size()>0 && eigenPts.size()>0)
    {


        sMatrix4 tf;
        teaser(descriptors,tf);
        //ransac(descriptors,tf);

        //_isam->addFrame(tf,cov);


        for(int i=0;i<good_matches.size();i++)
        {
            //cv::DMatch m( idx2,idx1,1 );
            cv::DMatch m=good_matches[i];
            int idx1=m.trainIdx;
            int idx2=m.queryIdx;

            FeatDescriptor d1=_descr[idx1];
            FeatDescriptor d2=descriptors[idx2];

            float3 p1=_points[idx1];
            float3 p2=keypoints[idx2];


            sMatrix3 cov1=d1.cov;
            sMatrix3 cov2=d2.cov;
            
            prev_corr.push_back(idx1);
            next_corr.push_back(idx2);
            
            int lidx=_isam->addLandmark(p1);
            _isam->connectLandmark(p1,lidx,prevFrame,cov1);
            _isam->connectLandmark(p2,lidx,-1,cov2);
        }        
#if 0
        sMatrix6 cov=calculatePoint2PointCov(keypoints,
                                        keypoints.size(),
                                        _points,
                                        _points.size(),
                                        next_corr,
                                        prev_corr,
                                        tf,
                                        params);
        //std::cout<<cov<<std::endl;
        _isam->addPoseConstrain(prevFrame,-1,tf,cov);
#endif
    }
    _descr=descriptors;



//    _isam->addPoseConstrain(-1,prevFrame,cov)
//    isam->add

    return true;

}

void keyptsMap::saveKeypoints(std::string fileName,const std::vector<float3> &keypts)
{
//    std::cout << "Saving keypoints to disk("<<fileName<<")..."<< std::endl;
    std::ofstream keypts_out_file(fileName, std::ios::out);

    for (int i = 0; i < keypts.size(); i++)
    {
#if 1
        float f[3];
        f[0]=float(keypts[i].x);
        f[1]=float(keypts[i].y);
        f[2]=float(keypts[i].z);

        keypts_out_file.write((char*)f, 3*sizeof(float));
#else
        keypts_out_file<<keypts[i].x<<" "
                       <<keypts[i].y<<" "
                       <<keypts[i].z<<"\n";
#endif
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
