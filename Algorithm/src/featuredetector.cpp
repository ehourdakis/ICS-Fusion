#include "featuredetector.h"
#include<iostream>
#include<fstream>

#include <opencv2/features2d.hpp>
#include"defs.h"
#include <algorithm>
FeatureDetector::FeatureDetector(kparams_t p, IcsFusion *f, PoseGraph *isam)
    :_fusion(f),
     _params(p),
     _isam(isam)
{

    int  	nfeatures = 500;
    int  	octaveLayers = 3;
    double  contrastThreshold = 0.01;
    double  edgeThreshold = 6;
    double  sigma = 1.6;

    focusThr=45;
    sift = cv::xfeatures2d::SIFT::create(nfeatures,
                                         octaveLayers,
                                         contrastThreshold,
                                         edgeThreshold,
                                         sigma);

    float cov_thr=10;
    covEstim=new SiftCovEstimator(_params.inputSize.y,_params.inputSize.x,cov_thr,true);
    drawnDesc=(uchar3*)malloc(sizeof(uchar3)*_params.inputSize.x*_params.inputSize.y);
    oldDrawnDesc=(uchar3*)malloc(sizeof(uchar3)*_params.inputSize.x*_params.inputSize.y);

    memset(drawnDesc,0,_params.inputSize.x*_params.inputSize.y*3);
    memset(oldDrawnDesc,0,_params.inputSize.x*_params.inputSize.y*3);

    oldFocusMeasure=-1;
}

Eigen::MatrixXd FeatureDetector::computeCov2DTo3D(Eigen::MatrixXd cov2D,
                                                  double depth,
                                                  double fx,
                                                  double fy,
                                                  double cx,
                                                  double cy,
                                                  double depth_noise_cov)
{

    Eigen::MatrixXd cov3D = Eigen::MatrixXd::Zero(3,3);


    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(3,2);
    F(0,0) = depth / fx;
    F(1,1) = depth / fy;

    Eigen::MatrixXd L = Eigen::MatrixXd::Zero(3,3);
    L(0,0) = - cx / fx;
    L(1,1) = - cy / fy;
    L(2,2) = 1.00;

    Eigen::MatrixXd Qz = Eigen::MatrixXd::Zero(3,3);
    Qz(2,2) = depth_noise_cov;

    cov3D.noalias() = F*cov2D*F.transpose();

    cov3D.noalias() += L*Qz*L.transpose();

    return cov3D;
}


void FeatureDetector::getFeatImage(uchar3 *out, std::vector<cv::DMatch> &good_matches)
{
    int s=_params.inputSize.x*_params.inputSize.y*3*2;

    if(oldCvKeypoints.size()==0 || good_matches.size()==0)
    {
        memset(out,0,s);
        return;
    }

    cv::Mat cvOldDesc(_params.inputSize.y, _params.inputSize.x, CV_8UC3,oldDrawnDesc);
    cv::Mat cvNewDesc(_params.inputSize.y, _params.inputSize.x, CV_8UC3,drawnDesc);

    cv::drawMatches( cvNewDesc, cvKeypoints, cvOldDesc, oldCvKeypoints, good_matches, cvOutput, cv::Scalar::all(-1),
                 cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::DEFAULT );


    memcpy(out,cvOutput.data,s);
}

void FeatureDetector::saveImage(char *filename) const
{
    imwrite( filename, cvOutput );
}

void FeatureDetector::getFeatImage(uchar3 *out)
{    
    covEstim->getDrawnData(out);
}

void FeatureDetector::calcMask(DepthHost &depth,cv::Mat &mask)
{
     mask=cv::Mat::ones(depth.size.y,depth.size.x, CV_8U );

     uint2 px;
     for(px.x=0;px.x<depth.size.x;px.x++)
     {
         for(px.y=0;px.y<depth.size.y;px.y++)
         {
             if(depth[px]<0.0001f ||depth[px]>4.00 || depth[px]!=depth[px])
             {
                 mask.at<uchar>(px.y,px.x,0)=0;
             }
         }
     }
}

void FeatureDetector::detectFeatures(int frame,
                                     DepthHost &depth, RgbHost &rgb,
                                     std::vector<float3> &keypts3D,
                                     std::vector<FeatDescriptor> &descr)
{
    (void) frame;
    keypts3D.clear();
    descr.clear();
    std::vector<cv::KeyPoint> allcvKeypoints;

    cv::Mat cvRgb=cv::Mat(_params.inputSize.y, _params.inputSize.x, CV_8UC3, rgb.data());
    cv::Mat cvGrey;
    cv::cvtColor(cvRgb, cvGrey, CV_BGR2GRAY);


    /*
    cv::Mat cvLaplacian;
    Laplacian(cvGrey, cvLaplacian, CV_8UC1, 3, 1, 0, cv::BORDER_DEFAULT);
    cv::Vec4d m, stdv;
    cv::meanStdDev(cvLaplacian, m, stdv);
    double focusMeasure = (stdv.val[0]*stdv.val[0]) / m.val[0];
    double diff=oldFocusMeasure-focusMeasure;
    if(diff<0)
        diff=-diff;
    std::cout<<"BLUR:"<<focusMeasure<<" "<<diff<<std::endl;


//    char buff[64];
//    sprintf(buff,"/tmp/f%d.png",frame);
//    imwrite( buff, cvLaplacian );

    //if(focusMeasure<focusThr || (oldFocusMeasure>0 && diff>6 )  )
    if(focusMeasure<focusThr )
        return;

    oldFocusMeasure=focusMeasure;
    */


    std::swap(oldDrawnDesc,drawnDesc);
    std::swap(oldCvKeypoints,cvKeypoints);


    cvKeypoints.clear();

    //store cv descriptors
    cv::Mat descrMat;
    cv::Mat mask;
    calcMask(depth,mask);


    sift->detectAndCompute(cvGrey,mask, allcvKeypoints,descrMat);

    keypts3D.reserve(allcvKeypoints.size());
    descr.reserve(allcvKeypoints.size());

    covEstim->load(cvGrey.data,cvRgb.data);

    Image<float3, Host> vertexes=_fusion->getAllVertex();
    for(uint i=0;i<allcvKeypoints.size();i++)
    {
        Eigen::Matrix2d cov2d;
        if( covEstim->calcCovariance(allcvKeypoints[i],cov2d) )
        //if(1)
        {
            cv::KeyPoint point=allcvKeypoints[i];

            uint2 px=make_uint2((uint) (point.pt.x+0.5),
                                (uint) (point.pt.y+0.5) );

            float3 vert=vertexes[px];

            FeatDescriptor d;
            fromCvMatRow(d,descrMat,i);
            d.s2=point.size/2;

            d.x=(float)point.pt.x;
            d.y=(float)point.pt.y;

            //std::cout<<cov2d<<std::endl;

            Eigen::MatrixXd eigenCov=computeCov2DTo3D(cov2d,
                                   vert.z,
                                   _params.camera.x,
                                   _params.camera.y,
                                   _params.camera.z,
                                   _params.camera.w,
                                   _params.cov_z);

            for(int i=0;i<3;i++)
            {
                for(int j=0;j<3;j++)
                {
                    d.cov(i,j)=eigenCov(i,j);
                }
            }

            cvKeypoints.push_back(point);
            keypts3D.push_back(vert);
            descr.push_back(d);
        }
    }

    covEstim->getDrawnData(drawnDesc);
    std::cout<<"Features Detected:"<<descr.size()<<std::endl;
    vertexes.release();

    mask.release();
    descrMat.release();
    cvGrey.release();
}
