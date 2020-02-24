#ifndef SIFT_WIHT_COV_H
#define SIFT_WIHT_COV_H

#include "cxcore.h"


/** holds feature data relevant to detection */
struct detection_data
{
	int r;
	int c;
	int octv;
	int intvl;
	double subintvl;
	double scl_octv;
	float* values;
	int nr_values;
};

// struct feature;

/** default number of sampled intervals per octave */
#define SIFT_INTVLS 3

/** default sigma for initial gaussian smoothing */
#define SIFT_SIGMA 1.6f

/** default threshold on keypoint contrast |D(x)| */
//#define SIFT_CONTR_THR 0.04
#define SIFT_CONTR_THR 0.04

/** default threshold on keypoint ratio of principle curvatures */
#define SIFT_CURV_THR 10

/** double image size before pyramid construction? */
#define SIFT_IMG_DBL 1

/** default width of descriptor histogram array */
#define SIFT_DESCR_WIDTH 4

/** default number of bins per histogram in descriptor array */
#define SIFT_DESCR_HIST_BINS 8

/* assumed gaussian blur for input image */
#define SIFT_INIT_SIGMA 0.5

/* width of border in which to ignore keypoints */
#define SIFT_IMG_BORDER 5

/* maximum steps of keypoint interpolation before failure */
#define SIFT_MAX_INTERP_STEPS 5

/* default number of bins in histogram for orientation assignment */
#define SIFT_ORI_HIST_BINS 36

/* determines gaussian sigma for orientation assignment */
#define SIFT_ORI_SIG_FCTR 1.5

/* determines the radius of the region used in orientation assignment */
#define SIFT_ORI_RADIUS 3.0 * SIFT_ORI_SIG_FCTR

/* number of passes of orientation histogram smoothing */
#define SIFT_ORI_SMOOTH_PASSES 2

/* orientation magnitude relative to max that results in new feature */
#define SIFT_ORI_PEAK_RATIO 0.8

/* determines the size of a single descriptor orientation histogram */
#define SIFT_DESCR_SCL_FCTR 3.0

/* threshold on magnitude of elements of descriptor vector */
#define SIFT_DESCR_MAG_THR 0.2

/* factor used to convert floating-point descriptor to unsigned char */
#define SIFT_INT_DESCR_FCTR 512.0

/* returns a feature's detection data */
#define feat_detection_data(f) ( (struct detection_data*)(f->feature_data) )


/*************************** Function Prototypes *****************************/

/**
Finds SIFT features in an image using default parameter values.  All
detected features are stored in the array pointed to by \a feat.

@param img the image in which to detect features
@param feat a pointer to an array in which to store detected features

@return Returns the number of features stored in \a feat or -1 on failure
@see _sift_features()
*/
int sift_features( IplImage* img, struct feature** feat );


int _sift_features(IplImage* img, struct feature** feat, int intvls,
						  double sigma, double contr_thr, int curv_thr,
						  int img_dbl, int descr_width, int descr_hist_bins);

/************************************************************************/

// OWN DECLARATIONS AND EXPORTS
IplImage*   create_init_img(IplImage*, int, double );
IplImage*** build_gauss_pyr(IplImage*, int, int, double);
IplImage*** build_dog_pyr( IplImage***, int, int);

CvSeq* scale_space_extrema(IplImage*** dog_pyr, 
                           int octvs, 
                           int intvls,
						   double contr_thr, 
                           int curv_thr, 
                           CvMemStorage* storage);

void calc_feature_scales(CvSeq* features, double sigma, int intvls);
void adjust_for_img_dbl(CvSeq* features);
int is_extremum( IplImage***, int, int, int, int );
feature* interp_extremum( IplImage***, int, int, int, int, int, double);
feature* new_feature( void );
int is_too_edge_like( IplImage*, int, int, int );
void calc_feature_oris( CvSeq*, IplImage*** );
void release_pyr( IplImage**** pyr, int octvs, int n );





#endif
