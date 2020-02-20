#include"harris.h"

Harris::Harris()
{
    blockSize = 3;
    apertureSize = 3;
    k = 0.099;
    thresh=150;
}

void Harris::detectCorners(VertHost &vert,
                      RgbHost &rgb,
                      std::vector<int> &points,
                      std::vector<float3> &keypts3D)
{
    points.clear();
    keypts3D.clear();

    cvRgb = cv::Mat(rgb.size.y, rgb.size.x, CV_8UC3, rgb.data());
    dst = cv::Mat::zeros( cvRgb.size(), CV_32FC1 );
    //convert image to grey scale
    cv::cvtColor(cvRgb, cvGrey, CV_BGR2GRAY);
    int blockSize=3;
    bool useHarrisDetector=true;
    double k=0.04;

    cv::goodFeaturesToTrack(cvGrey,
                            corners,
                            200,
                            0.001,
                            2,
                            cv::Mat(),
                            blockSize,
                            useHarrisDetector,
                            k );

    dst_norm=dst;

    for(int i=0;i<corners.size();i++)
    {
        cv::Point2f corner=corners[i];
        uint2 pix=make_uint2(corner.x,corner.y);

        int idx=pix.x*rgb.size.y+pix.y;
        float3 v=vert[pix];
        if(v.x!=0.0 || v.y!=0.0 || v.z!=0.0)
        {
            points.push_back(idx);
            keypts3D.push_back(v);
        }
    }
}

void Harris::showKeypts(cv::Mat &outMat)
{
    outMat=cvRgb.clone();

    for(int i=0;i<corners.size();i++)
    {
        cv::Point2f corner=corners[i];
        cv::circle( outMat, corner, 5,  cv::Scalar(255,0,0), 2, 8, 0 );
    }

}
