#include "featuredetector.h"
#include<iostream>
#include<fstream>

#include <opencv2/features2d.hpp>

FeatureDetector::FeatureDetector(kparams_t p, IcsFusion *f, PoseGraph *isam)
    :_fusion(f),
     _params(p),
     _isam(isam)
{

    int  	nfeatures = 100;
    int  	octaveLayers = 3;
    double  contrastThreshold = 0.04;
    double  edgeThreshold = 10;
    double  sigma = 1.6;

    sift = cv::xfeatures2d::SIFT::create(nfeatures,
                                         octaveLayers,
                                         contrastThreshold,
                                         edgeThreshold,
                                         sigma);

    //sift = cv::xfeatures2d::SIFT::create(0,3,0.04,10,1.6);

//    matcher = cv::FlannBasedMatcher::create();
    drawNewData=false;
    //TODO add ratio_thresh to params
    ratio_thresh = 0.7f;
}

void FeatureDetector::getFeatImage(cv::Mat &outMat)
{
//    outMat=cvGrey.clone();
    cv::drawKeypoints(cvGrey,cvKeypoints,outMat,cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
}

void FeatureDetector::getFeatImage(uchar3 *out)
{
    int s=_params.inputSize.x*_params.inputSize.y*3;
    if(cvKeypoints.size()==0 )
    {
        memset(out,0,s);
        return;
    }

    if(drawNewData)
    {
        //cv::drawKeypoints(cvGrey, cvKeypoints, cvOutput,cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::drawKeypoints(cvGrey,cvKeypoints,cvOutput,cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        drawNewData=false;
    }
    memcpy(out,cvOutput.data,s);
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
//             mask.at<uchar>(px.y,px.x,0)=0;
             if(depth[px]<0.0001f ||depth[px]>4 || depth[px]!=depth[px])
             {
                 mask.at<uchar>(px.y,px.x,0)=0;
             }

         }
     }
}

void FeatureDetector::detectFeatures(int frame, DepthHost &depth, RgbHost &rgb,
                                                  std::vector<float3> &keypts3D,
                                                  std::vector<FeatDescriptor> &descr)
{    
//    std::cout<<"Feature detection"<<std::endl;
    //load rgb image to opencv
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

    keypts3D.reserve(cvKeypoints.size());
    descr.reserve(cvKeypoints.size());

    Image<float3, Host> vertexes=_fusion->getAllVertex();
    for(uint i=0;i<cvKeypoints.size();i++)
    {
        uint2 px=make_uint2((uint) (cvKeypoints[i].pt.x+0.5),
                            (uint) (cvKeypoints[i].pt.y+0.5) );

        if(px.x>=vertexes.size.x ||px.y>=vertexes.size.y)
            continue;

        float3 vert=vertexes[px];
        float distance=l2(vert);        

        //eliminate too small or big values.
        //Posible depth sensor error.
        if(distance<0.0001f ||distance>4)
            continue;


        FeatDescriptor d;        
        fromCvMatRow(d,descrMat,i);
        d.s2=sq(cvKeypoints[i].size/2);
        d.x=(float)cvKeypoints[i].pt.x;
        d.y=(float)cvKeypoints[i].pt.y;

//        float2 pf;
//        pf.x=cvKeypoints[i].pt.x;
//        pf.y=cvKeypoints[i].pt.y;

        d.cov=sMatrix3();
        //d.cov=d.cov*d.s2;
        d.cov=d.cov*1e-6;

        //sMatrix3 cov=calcCovariance(vertexes,pf,cvKeypoints[i].size/2,vert);
        keypts3D.push_back(vert);
//        float f=infNorm(d.cov);
//        std::cout<<"C "<<f<<" "<<d.s2<<std::endl;
//        std::cout<<d.cov<<std::endl;

        descr.push_back(d);
    }
    std::cout<<"Features Detected:"<<descr.size()<<std::endl;

    drawNewData=true;
    vertexes.release();
}

void FeatureDetector::getDescrFromMat(int row,cv::Mat &mat,FeatDescriptor &descr)
{
    for(int i=0;i<DESCR_SIZE;i++)
    {
        descr.data[i]=mat.at<double>(row,i);
    }
}

void FeatureDetector::writeWrongNormPoints(char *fileName)
{
    Image<float3, Host> vert=_fusion->getAllVertex();
    Image<TrackData, Host> trackData=_fusion->getTrackData();
    uint keyptsSize=_fusion->getWrongNormalsSize();

    using namespace std;
    ofstream file(fileName, std::ios::out);

    file<<"VERSION 0.7\n";
    file<<"FIELDS x y z\n";
    file<<"SIZE 4 4 4\n";
    file<<"TYPE F F F\n";
    file<<"COUNT 1 1 1\n";
    file<<"WIDTH "<<1<<"\n";
    file<<"HEIGHT "<<keyptsSize<<"\n";

    file<<"VIEWPOINT 0 0 0 1 0 0 0\n";
    file<<"POINTS "<<keyptsSize<<"\n";
    file<<"DATA ascii"<<endl;//also flush

    uint2 px;
    for(px.x=0;px.x<_params.computationSize.x;px.x++)
    {
        for(px.y=0;px.y<_params.computationSize.y;px.y++)
        {
            TrackData &data = trackData[px];
            if(data.result==-5)
            {
                file<<vert[px].x<<" "<<vert[px].y<<" "<<vert[px].z<<"\n";
            }
        }
    }
    file<<endl;
    file.close();

    vert.release();
    trackData.release();

}

void FeatureDetector::writeWrongNormNormals(char *fileName)
{
    Image<float3, Host> norm=_fusion->getAllNormals();
    Image<TrackData, Host> trackData=_fusion->getTrackData();
    uint keyptsSize=_fusion->getWrongNormalsSize();

    using namespace std;
    ofstream file(fileName, std::ios::out);

    file<<"VERSION 0.7\n";
    file<<"FIELDS x y z\n";
    file<<"SIZE 4 4 4\n";
    file<<"TYPE F F F\n";
    file<<"COUNT 1 1 1\n";
    file<<"WIDTH "<<1<<"\n";
    file<<"HEIGHT "<<keyptsSize<<"\n";

    file<<"VIEWPOINT 0 0 0 1 0 0 0\n";
    file<<"POINTS "<<keyptsSize<<"\n";
    file<<"DATA ascii"<<endl;//also flush


    uint2 px;
    for(px.x=0;px.x<_params.computationSize.x;px.x++)
    {
        for(px.y=0;px.y<_params.computationSize.y;px.y++)
        {
            TrackData &data = trackData[px];
            if(data.result==-5)
            {
                file<<norm[px].x<<" "<<norm[px].y<<" "<<norm[px].z<<"\n";
            }
        }
    }
    file<<endl;
    file.close();

    norm.release();
    trackData.release();
}

void FeatureDetector::writePoints(std::vector<float3> &vec,char *fileName)
{
    using namespace std;
    ofstream file(fileName, std::ios::out);

//    file<<"VERSION 0.7\n";
//    file<<"FIELDS x y z\n";
//    file<<"SIZE 4 4 4\n";
//    file<<"TYPE F F F\n";
//    file<<"COUNT 1 1 1\n";
//    file<<"WIDTH "<<1<<"\n";
//    file<<"HEIGHT "<<vec.size()<<"\n";

//    file<<"VIEWPOINT 0 0 0 1 0 0 0\n";
//    file<<"POINTS "<<vec.size()<<"\n";
//    file<<"DATA ascii"<<endl;//also flush

    for(uint i=0;i<vec.size();i++)
    {
        file<<vec[i].x<<" "<<vec[i].y<<" "<<vec[i].z<<"\n";
    }
    file<<endl;
    file.close();
}

void FeatureDetector::writePointsPcd(Image<float3, Host> &vertex,
              Image<float3, Host> &norm,
              char *fileName)
{
    using namespace std;
    ofstream file(fileName, std::ios::out);

    file<<"VERSION 0.7\n";
    file<<"FIELDS x y z\n";
    file<<"SIZE 4 4 4\n";
    file<<"TYPE F F F\n";
    file<<"COUNT 1 1 1\n";
    file<<"WIDTH "<<vertex.size.x<<"\n";
    file<<"HEIGHT "<<vertex.size.y<<"\n";

    file<<"VIEWPOINT 0 0 0 1 0 0 0\n";
    file<<"POINTS "<<vertex.size.x*vertex.size.y<<"\n";
    file<<"DATA ascii"<<endl;//also flush

    for(uint x=0;x<vertex.size.x;x++)
    {
        for(uint y=0;y<vertex.size.y;y++)
        {
            uint2 px=make_uint2(x,y);
            file<<vertex[px].x<<" "<<vertex[px].y<<" "<<vertex[px].z<<"\n";
//            file<<norm[px].x<<" "<<norm[px].y<<" "<<norm[px].z<<"\n";
        }
    }
    file<<endl;
    file.close();
}

void FeatureDetector::writeNormalsPcd(Image<float3, Host> &vertex,
              Image<float3, Host> &norm,
              char *fileName)
{
    using namespace std;
    ofstream file(fileName, std::ios::out);    

    file<<"VERSION 0.7\n";
    file<<"FIELDS normal_x normal_y normal_z\n";
    file<<"SIZE 4 4 4\n";
    file<<"TYPE F F F\n";
    file<<"COUNT 1 1 1\n";
    file<<"WIDTH "<<vertex.size.x<<"\n";
    file<<"HEIGHT "<<vertex.size.y<<"\n";

    file<<"VIEWPOINT 0 0 0 1 0 0 0\n";
    file<<"POINTS "<<vertex.size.x*vertex.size.y<<"\n";
    file<<"DATA ascii"<<endl;//also flush

    for(uint x=0;x<vertex.size.x;x++)
    {
        for(uint y=0;y<vertex.size.y;y++)
        {
            uint2 px=make_uint2(x,y);
//            file<<vertex[px].x<<" "<<vertex[px].y<<" "<<vertex[px].z<<" ";
            file<<norm[px].x<<" "<<norm[px].y<<" "<<norm[px].z<<"\n";
        }
    }
    file<<endl;
    file.close();
}
