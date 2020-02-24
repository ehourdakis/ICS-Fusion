#include "sift_cov.h"
#include "sift.h"
#include <opencv2/features2d.hpp>
/** Convert SURF feature points found by openCV to Ipoint structure
 */
void cvSeq2Ipts(const CvSeq* keypoints, const CvSeq* descriptors, IpVec& ipts)
{

	// from cv.h
	typedef struct CvSURFPoint
	{
		CvPoint2D32f pt;
		int laplacian;
		int size;
		float dir;
		float hessian;
	} CvSURFPoint;
	

	// empty vector of interest points
	ipts.clear();

	CvSURFPoint* surfPt;
	float* descr;
	Ipoint* ipt;
	for( int k = 0; k < keypoints->total; k++ ) {
		
		// get point from sequence
		surfPt = (CvSURFPoint*) cvGetSeqElem(keypoints, k);

		// discard if scale value is impossible
		if(surfPt->size < 9)
			continue;
		
		// create new interest point and assign values
		ipt = new Ipoint();
		ipt->x			 = surfPt->pt.x;
		ipt->y			 = surfPt->pt.y;
		ipt->laplacian	 = surfPt->laplacian;
		ipt->orientation = surfPt->dir;
		ipt->scale		 = (float) (1.2/9.0 * surfPt->size);

		// assign descriptor data
		descr = (float*) cvGetSeqElem(descriptors, k);
		ipt->initializeDescr(SURF_DESCR_SIZE);
		for( int i = 0; i < SURF_DESCR_SIZE; i++ )
			ipt->descriptor[i] = descr[i];

		// add to vector of interest points
		ipts.push_back(*ipt);
	}
}

/** Convert SIFT feature points found by Rob Hess SIFT implementation to Ipoint structure
 */
void feature2Ipts(struct feature* features, int nrFeat, IpVec& ipts)
{
	ipts.clear();
	Ipoint* ipt;
    for( int k = 0; k < nrFeat; k++ )
    {
		// create new interest point and assign values
		ipt = new Ipoint();
		ipt->x			 = (float) features[k].x;
		ipt->y			 = (float) features[k].y;
		ipt->laplacian	 = 0;
		ipt->orientation = (float) features[k].ori;
		ipt->scale		 = (float) features[k].scl;

		// assign descriptor data
		ipt->initializeDescr(features[k].d);
		for( int i = 0; i < features[k].d; i++ )
			ipt->descriptor[i] = (float) features[k].descr[i];

		// add to vector of interest points
		ipts.push_back(*ipt);
	}
}

SiftCov::SiftCov(const char *sfilename, bool draw)
    :ipts(DETECTOR_SIFT),
    frame(nullptr),
    drawFrame(nullptr),
    imgPyr(nullptr),
    octaves(0),
    estimator(nullptr),
    intervals()
{
    frame = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    if( !frame )
    {
        cout << "COV Error: Unable to load image from " << filename << "\n";
        exit( 1 );
    }
}

SiftCov::SiftCov()
    :ipts(DETECTOR_SIFT),
    frame(nullptr),
    drawFrame(nullptr),
    imgPyr(nullptr),
    estimator(nullptr),
    octaves(0),
    intervals()
{

}

void SiftCov::load(int width, int height, void *data,void *rgb)
{
    CvSize size = cvSize(height,width);
    frame = cvCreateImageHeader(size, IPL_DEPTH_8U,1);
    frame->imageData =(char*) data;

    drawFrame = cvCreateImageHeader(size, IPL_DEPTH_8U,3);
    drawFrame->imageData =(char*)malloc(height*width*3);
    memcpy(drawFrame->imageData ,rgb,height*width*3);


//    showImage("Uncertainty Estimation: keypoints and their covariance", frame);

//    if( !frame )
//    {
//        cout << "COV Error: Unable to load image from " << filename << "\n";
//        exit( 1 );
//    }

    // create image pyramid (modified functions from SIFT implementation)
    IplImage* init_img = create_init_img( frame, SIFT_IMG_DBL, SIFT_SIGMA );
    octaves = static_cast<int>(log(static_cast<float>(MIN( init_img->width, init_img->height )) ) / log(2.0f)) - 2;
    intervals = 3;
    IplImage*** gauss_pyr = build_gauss_pyr( init_img, octaves, intervals, SIFT_SIGMA );
    imgPyr = build_dog_pyr( gauss_pyr, octaves, intervals );
    release_pyr( &gauss_pyr, octaves, intervals + 3 );

    if(estimator!=nullptr)
        delete estimator;

    estimator=new CovEstimator( (const IplImage***) imgPyr, DETECTOR_SIFT, octaves, intervals );
}

unsigned int SiftCov::detectFeatures()
{
    struct feature* features = nullptr;
    int n = _sift_features(frame, &features, SIFT_INTVLS, SIFT_SIGMA, SIFT_THRES,
                    SIFT_CURV_THR, SIFT_IMG_DBL, SIFT_DESCR_WIDTH, SIFT_DESCR_HIST_BINS);
    // convert found points to Ipoint structure
    feature2Ipts(features, n, ipts);
    delete [] features;

    for(unsigned int k = 0; k < ipts.size(); k++ )
    {
        ipts[k].cov = cvCloneMat( estimator->getCovAt( ipts.at(k).x, ipts.at(k).y, ipts.at(k).scale ) );
        estimator->drawCovInto( drawFrame, static_cast<int>(ipts.at(k).x), static_cast<int>(ipts.at(k).y));
    }

    return ipts.size();
}

void SiftCov::draw(int width, int height, void *data)
{

    memcpy(data,drawFrame->imageData,width*height*3);
}

void SiftCov::getCovariance() 
{
    /*** Creating image pyramid ***/
    int octaves=0, intervals=0;
    IplImage*** imgPyr=NULL;

    // create image pyramid (modified functions from SIFT implementation)
    IplImage* init_img = create_init_img( frame, SIFT_IMG_DBL, SIFT_SIGMA );
    octaves = static_cast<int>(log(static_cast<float>(MIN( init_img->width, init_img->height )) ) / log(2.0f)) - 2;
    intervals = 3;
    IplImage*** gauss_pyr = build_gauss_pyr( init_img, octaves, intervals, SIFT_SIGMA );
    imgPyr = build_dog_pyr( gauss_pyr, octaves, intervals );
    release_pyr( &gauss_pyr, octaves, intervals + 3 );

    //ofstream& outfile = GeneralFeature::initializeFile("covs.kps", ipts);

    // draw and write the data at the same time to reduce memory usage for matrix creation
    estimator=new CovEstimator( (const IplImage***) imgPyr, DETECTOR_SIFT, octaves, intervals );

//    if( bDraw)	drawFrame = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_COLOR);

    // Do the covariance estimation
    for(unsigned int k = 0; k < ipts.size(); k++ )
    {
        ipts[k].cov = estimator->getCovAt( ipts.at(k).x, ipts.at(k).y, ipts.at(k).scale );
// 		  CvMat* cov = estimator->getCovAt( ipts.at(k).x, ipts.at(k).y, ipts.at(k).scale );
//        if( bDraw) estimator->drawCovInto( drawFrame, static_cast<int>(ipts.at(k).x), static_cast<int>(ipts.at(k).y));
//        GeneralFeature::write(outfile, ipts.at(k), ipts[k].cov);
    }

//    GeneralFeature::closeFile(outfile);

//    /*** Displaying image with covariances ***/
    
//    if( bDraw) showImage("Uncertainty Estimation: keypoints and their covariance", drawFrame);
//    cvWaitKey(0);
}
