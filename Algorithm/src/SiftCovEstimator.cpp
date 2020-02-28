#include"SiftCovEstimator.h"
#include <opencv2/core/types.hpp>

SiftCovEstimator::SiftCovEstimator(int width,int height,float thr,bool draw)
    :_width(width),
     _height(height),
     _thr(thr),
     _draw(draw),
     imgPyr(nullptr),
     _estimator(nullptr)
{
    CvSize size = cvSize(height,width);
    _frame = cvCreateImageHeader(size, IPL_DEPTH_8U,1);

    if(draw)
    {
        _drawFrame = cvCreateImageHeader(size, IPL_DEPTH_8U,3);
        _drawFrame->imageData =(char*)malloc(height*width*3);
    }
    else
    {
        _drawFrame=nullptr;
    }
}

void SiftCovEstimator::clear()
{
    delete _frame;
    _frame=nullptr;
    cvReleaseImage(&_drawFrame);
    delete _drawFrame;
    _drawFrame=nullptr;

    if(imgPyr!=nullptr)
        release_pyr( &imgPyr, octaves, intervals + 2 );
}

void SiftCovEstimator::load(void *data,void *rgb)
{
    _frame->imageData =(char*) data;

    if(_drawFrame!=nullptr)
        memcpy(_drawFrame->imageData ,rgb,_height*_width*3);


    if(imgPyr!=nullptr)
         release_pyr(&imgPyr,octaves,intervals + 2);



    // create image pyramid (modified functions from SIFT implementation)
    IplImage* init_img = create_init_img( _frame, SIFT_IMG_DBL, SIFT_SIGMA );
    octaves = static_cast<int>(log(static_cast<float>(MIN( init_img->width, init_img->height )) ) / log(2.0f)) - 2;
    intervals = 3;
    IplImage*** gauss_pyr = build_gauss_pyr( init_img, octaves, intervals, SIFT_SIGMA );
    imgPyr = build_dog_pyr( gauss_pyr, octaves, intervals );
    release_pyr( &gauss_pyr, octaves, intervals + 3 );


    if(_estimator!=nullptr)
    {
        delete _estimator;
    }

    _estimator=new CovEstimator( (const IplImage***) imgPyr, DETECTOR_SIFT, octaves, intervals );

    cvReleaseImage(&init_img);
//    release_pyr(&imgPyr,octaves,intervals);

}

bool SiftCovEstimator::calcCovariance(const cv::KeyPoint &point, Eigen::Matrix2d &cov)
{
    float scale=point.size/2;
    CvMat *m=_estimator->getCovAt( point.pt.x, point.pt.y, scale);
    float infNorm=cv2SMatrix(m,cov);

    if(infNorm<_thr)
    {
        _estimator->drawCovInto( _drawFrame, point.pt.x, point.pt.y);
        return true;
    }
    return false;
}

float SiftCovEstimator::cv2SMatrix(CvMat *m, Eigen::Matrix2d &cov2D)
{
    float _max=0;
    for(int i=0;i<2;i++)
    {
        for(int j=0;j<2;j++)
        {
            float el=CV_MAT_ELEM(*m, float, i, j);
            cov2D(i,j)=el;

            if( abs(el)>_max)
                _max=abs(el);
        }
    }
    return _max;

}

void SiftCovEstimator::getDrawnData(void *out)
{
    memcpy(out,_drawFrame->imageData,_width*_height*3);
}
