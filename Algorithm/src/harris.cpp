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

    cvRgb = cv::Mat(vert.size.y, vert.size.x, CV_8UC3, rgb.data());
    dst = cv::Mat::zeros( cvRgb.size(), CV_32FC1 );
    //convert image to grey scale

    cv::cvtColor(cvRgb, cvGrey, CV_BGR2GRAY);

    cv::cornerHarris( cvGrey,dst,blockSize, apertureSize, k );
    cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
    //cv::convertScaleAbs( dst_norm, dst_norm_scaled );

    std::cout<<"H:"<<dst_norm.rows <<" "<< dst_norm.cols<<std::endl;

    uint2 pix;
    int idx=0;
    for(pix.x=0;pix.x<dst_norm.rows;pix.x++)
    {
        for(pix.y=0;pix.y<dst_norm.cols;pix.y++)
        {
            if( (int)dst_norm.at<float>(pix.x,pix.y) > thresh )
            {
                //circle( dst_norm_scaled, Point(j,i), 5,  Scalar(0), 2, 8, 0 );
                //keyVert[i]=vertices[pix];
                points.push_back(idx);
                //keypts3D.push_back(vert[pix]);
                idx++;
            }
        }
    }
}

void Harris::showKeypts(cv::Mat &outMat)
{
    outMat=cvRgb.clone();
    //cv::convertScaleAbs( dst_norm, outMat );
    for( int i = 0; i < dst_norm.rows ; i++ )
    {
        for( int j = 0; j < dst_norm.cols; j++ )
        {
            if( (int) dst_norm.at<float>(i,j) > thresh )
            {
                cv::circle( outMat, cv::Point(j,i), 5,  cv::Scalar(255,0,0), 2, 8, 0 );
            }
        }
    }
}
