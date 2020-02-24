#include "featuredetector.h"
#include<iostream>
#include<fstream>

#include <opencv2/features2d.hpp>
#include"defs.h"

FeatureDetector::FeatureDetector(kparams_t p, IcsFusion *f, PoseGraph *isam)
    :_fusion(f),
     _params(p),
     _isam(isam)
{

    int  	nfeatures = 400;
    int  	octaveLayers = 3;
    double  contrastThreshold = 0.01;
    double  edgeThreshold = 3;
    //double  sigma = 1.6;
    double  sigma = 1.6;

    //sift = cv::xfeatures2d::SIFT::create(0,3,0.04,10,1.6);

//    matcher = cv::FlannBasedMatcher::create();
    drawNewData=false;
    //TODO add ratio_thresh to params
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
// #ifndef DRAW_MATCHES
//     outMat=cvGrey.clone();
//     cv::drawKeypoints(cvGrey,cvKeypoints,outMat,cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//     return;
// #else

//    int s=_params.inputSize.x*_params.inputSize.y*3*2;
//    if(oldCvRgb.empty()||good_matches.size()==0)
//    {
//        memset(out,0,s);
//        return;
//    }

//    cv::drawMatches( cvRgb, cvKeypoints, oldCvRgb, oldCvKeypoints, good_matches, cvOutput, cv::Scalar::all(-1),
//                 cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::DEFAULT );
//// #endif
//    memcpy(out,cvOutput.data,s);
}

void FeatureDetector::saveImage(char *filename) const
{
    imwrite( filename, cvOutput );
}

void FeatureDetector::getFeatImage(uchar3 *out)
{    
    int s=_params.inputSize.x*_params.inputSize.y;
    memcpy(out,cvRgb.data,s*3);
    siftCov.draw(_params.inputSize.x,_params.inputSize.y,out);

}

float infNorm(const sMatrix3 &mat)
{
    float ret=-1;
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            float tmp=mat(i,j);
            if(tmp<0)
                tmp*=-1;
            if(tmp>ret)
                ret=tmp;
        }
    }
    return ret;
}

sMatrix3 calcCovariance(Image<float3, Host> vertexes,float2 pt,float r,float3 &avg)
{
    sMatrix3 ret;
    float2 ipt;
    //float3 avg=make_float3(0,0,0);
    int count=0;
    for(ipt.x=pt.x-r;ipt.x<=pt.x+r;ipt.x++)
    {
        if(ipt.x<0 ||pt.x>=vertexes.size.x)
            continue;

        for(ipt.y=pt.y-r;ipt.y<=pt.y+r;ipt.y++)
        {
            if(ipt.y<0 ||pt.y>=vertexes.size.y)
                continue;

            float xx=sq(ipt.x-pt.x);
            float yy=sq(ipt.y-pt.y);
            float dist=sqrt(xx+yy);
            if(dist>r)
                continue;

            uint2 px=make_uint2((uint) (ipt.x+0.5),
                                (uint) (ipt.y+0.5) );
            if(px.x>=vertexes.size.x || px.y>=vertexes.size.y)
                continue;

            float3 vert=vertexes[px];
            avg=avg+vert;
            count++;
        }
    }
    avg.x/=count;
    avg.y/=count;
    avg.z/=count;

    uint2 px=make_uint2((uint) (pt.x+0.5),
                        (uint) (pt.y+0.5) );

//    avg=vertexes[px];

    float s[3];
    s[0]=0;
    s[2]=0;
    s[1]=0;

    for(ipt.x=pt.x-r;ipt.x<=pt.x+r;ipt.x++)
    {
        if(ipt.x<0 ||pt.x>=vertexes.size.x)
            continue;

        for(ipt.y=pt.y-r;ipt.y<=pt.y+r;ipt.y++)
        {
            if(ipt.y<0 ||pt.y>=vertexes.size.y)
                continue;

            float xx=sq(ipt.x-pt.x);
            float yy=sq(ipt.y-pt.y);
            float dist=sqrt(xx+yy);
            if(dist>r)
                continue;

            uint2 px=make_uint2((uint) (ipt.x+0.5),
                                (uint) (ipt.y+0.5) );

            if(px.x>=vertexes.size.x || px.y>=vertexes.size.y)
                continue;
            float3 vert=vertexes[px];

            s[0]+=sq(vert.x-avg.x);
            s[1]+=sq(vert.y-avg.y);
            s[2]+=sq(vert.z-avg.z);
        }
    }

    s[0]=sqrt(s[0]/count);
    s[1]=sqrt(s[1]/count);
    s[2]=sqrt(s[2]/count);

    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            ret(i,j)=s[i]*s[j];
            if(i==j && ret(i,j)==0)
                ret(i,j)=1e-5;
        }
    }


    return ret;
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
    cvRgb = cv::Mat(_params.inputSize.y, _params.inputSize.x, CV_8UC3, rgb.data());
    cv::cvtColor(cvRgb, cvGrey, CV_BGR2GRAY);

    uint width=_params.inputSize.y;
    uint height=_params.inputSize.x;
    siftCov.load(width,height,cvGrey.data,cvRgb.data);

    siftCov.detectFeatures();
    IpVec vec=siftCov.getFeatures();


    std::cout<<"V:"<<vec.size()<<std::endl;

    Image<float3, Host> vertexes=_fusion->getAllVertex();
    for(int i=0;i<vec.size();i+=2)
    {
        Ipoint pt=vec[i];
        std::cout<<"PT:"<<pt.x<<" "<<pt.y<<std::endl;

        uint2 px=make_uint2((uint) (pt.x+0.5),
                            (uint) (pt.y+0.5) );

        if(depth[px]>0.0001f && depth[px]<4.00 && depth[px]==depth[px])
        {
            float depthVal=depth[px];
            float3 vert=vertexes[px];
            keypts3D.push_back(vert);

            FeatDescriptor desc;
            memcpy(desc.data,pt.descriptor,sizeof(float)*DESCR_SIZE);

            Eigen::Matrix2d cov2d;
            for(int i=0;i<2;i++)
            {
                for(int j=0;j<2;j++)
                {
                    cov2d(i,j)=CV_MAT_ELEM(*pt.cov, float, i, j) ;
                }
            }
            std::cout<<cov2d<<std::endl;

            Eigen::MatrixXd eigenCov=computeCov2DTo3D(cov2d,depthVal,
                             _params.camera.x,
                             _params.camera.y,
                             _params.camera.z,
                             _params.camera.w,
                             _params.cov_z);

            for(int i=0;i<2;i++)
            {
                for(int j=0;j<2;j++)
                {
                    desc.cov(i,j)=eigenCov(i,j);
                }
            }
            descr.push_back(desc);
        }
        cvReleaseMat(&pt.cov);
    }

    std::cout<<"Features Detected:"<<descr.size()<<std::endl;

    drawNewData=true;
    vertexes.release();

    return;



#if 0
//    std::cout<<"Feature detection"<<std::endl;
    //load rgb image to opencv
    oldCvRgb=cvRgb;
    oldCvKeypoints=cvKeypoints;

    cvRgb = cv::Mat(_params.inputSize.y, _params.inputSize.x, CV_8UC3, rgb.data());

    //convert image to grey scale
    cv::cvtColor(cvRgb, cvGrey, CV_BGR2GRAY);

    //cv keypoint type
    //std::vector<cv::KeyPoint> cvKeypoints;

    //store cv descriptors
    cv::Mat descrMat;
    //detect sift features
    cvKeypoints.clear();

    cv::Mat mask;
    calcMask(depth,mask);

    sift->detectAndCompute(cvGrey,mask, cvKeypoints,descrMat);

        std::cout<<descrMat.rows<<" "<<descrMat.cols<<std::endl;

    
    keypts3D.reserve(cvKeypoints.size());
    descr.reserve(cvKeypoints.size());

    Image<float3, Host> vertexes=_fusion->getAllVertex();
    for(uint i=0;i<cvKeypoints.size();i++)
    {
        uint2 px=make_uint2((uint) (cvKeypoints[i].pt.x+0.5),
                            (uint) (cvKeypoints[i].pt.y+0.5) );

        float3 vert=vertexes[px];    

        FeatDescriptor d;        
        fromCvMatRow(d,descrMat,i);
        d.s2=cvKeypoints[i].size;
        //d.s2=(cvKeypoints[i].size/2)/10000;

//         d.s2*=1e-4;

        d.x=(float)cvKeypoints[i].pt.x;
        d.y=(float)cvKeypoints[i].pt.y;

        d.cov=sMatrix3();
        d.cov=d.cov*d.s2;
        //d.cov=d.cov;

        keypts3D.push_back(vert);
        descr.push_back(d);
    }
#endif
}
