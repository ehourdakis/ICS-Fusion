/******************************************************************************
*  --- COVARIANCE ESTIMATOR ---                                               *
*                                                                             *
*  Copyright 2009 Bernhard Zeisl, Pierre Fite Georgel                         *
*                                                                             *
*  This program is free software: you can redistribute it and/or modify it    *
*  under the terms of the GNU General Public License as published by the Free *
*  Software Foundation, version 3 of the License.                             *
*  This program is distributed in the hope that it will be useful, but        *
*  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
*  or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License    *
*  (LISENCE.txt) and the disclaimer (DISCLAIMER.txt) for more details.        *
*                                                                             *
*  The software was written by Bernhard Zeisl <bernhard.zeisl@mytum.de and    *
*  Pierre Fite Georgel <pierre.georgel@gmail.com>.                            *
*  For more information please contact the project website at:                *
*  http://campar.in.tum.de/Main/CovarianceEstimator                           *
*                                                                             *
*  The software package makes use of three additional libraries:              *
*  - The openCV computer vision library http://opencv.willowgarage.com/       *
*    published under the open source BSD license.                             *
*  - The SIFT implementation from Rob Hess <hess@eecs.oregonstate.edu>.       *
*    please see http://web.engr.oregonstate.edu/~hess/                        *
*    The package is distributed under GNU GPL v2 exept for the SIFT featur    *
*    detection code. For more information on SIFT licensing see LISENCE.ubc.  *
*  - The openSURF library from http://code.google.com/p/opensurf1/            *
*    The package is distributed under GNU GPL v3.                             *
*                                                                             *
*  If you use our software, please cite following publication:                *
*  Zeisl, B. et.al. "Estimation of Location Uncertainty for Scale Invariant   *
*    Feature Points", Proc. of the British Maschine Vision Conference (BMCV)  *
*    2009                                                                     *
*                                                                             *
*  @INPROCEEDINGS{ zeisl09elu,                                                *
*    AUTHOR={Bernhard Zeisl and Pierre Fite Georgel and Florian Schweiger and *
*            Eckehard Steinbach and Nassir Navab},                            *
*    TITEL={{Estimation of Location Uncertainty for Scale Invariant Feature   *
*            Points}},                                                        *
*    BOOKTITLE={Proc. of the British Maschine Vision Conference (BMCV)},      *
*    YEAR={2009}                                                              *
*  }                                                                          *
*                                                                             *
*******************************************************************************/

#include "ImagePyramid.h"

#include <opencv2/features2d.hpp>
struct CvSurfHF
{
    int p0, p1, p2, p3;
    float w;
};

void icvResizeHaarPattern( const int src[][5], CvSurfHF* dst, int n, int oldSize, int newSize, int widthStep )
{
    for( int k = 0; k < n; k++ )
    {
        int dx1 = src[k][0]*newSize/oldSize;
        int dy1 = src[k][1]*newSize/oldSize;
        int dx2 = src[k][2]*newSize/oldSize;
        int dy2 = src[k][3]*newSize/oldSize;
        dst[k].p0 = dy1*widthStep + dx1;
        dst[k].p1 = dy2*widthStep + dx1;
        dst[k].p2 = dy1*widthStep + dx2;
        dst[k].p3 = dy2*widthStep + dx2;
        dst[k].w = src[k][4]/((float)(dx2-dx1)*(dy2-dy1));
    }
}

inline float icvCalcHaarPattern( const int* origin, const CvSurfHF* f, int n )
{
    double d = 0;
    for( int k = 0; k < n; k++ )
        d += (origin[f[k].p0] + origin[f[k].p3] - origin[f[k].p1] - origin[f[k].p2])*f[k].w;
    return (float)d;
}


/** Calculation of the scale spave image pyramid
 *  Modified code from openCV cvsurf.cpp 
 */
void ImagePyramid::create(IplImage* img, int octaves, int layers) {

	this->octaves = octaves;
	this->intervals = layers;

    /* Wavelet size at first layer of first octave. */ 
    const int HAAR_SIZE0 = 9;    

    /* Wavelet size increment between layers. This should be an even number, 
       such that the wavelet sizes in an octave are either all even or all odd.
       This ensures that when looking for the neighbours of a sample, the layers
       above and below are aligned correctly. */
    const int HAAR_SIZE_INC = 6; 

    /* Sampling step along image x and y axes at first octave. This is doubled
       for each additional octave. WARNING: Increasing this improves speed, 
       however keypoint extraction becomes unreliable. */
    const int SAMPLE_STEP0 = 1; 

	/* Integral image */
	CvMat* sum;
	if( img->depth != 64 )
		sum = cvCreateMat( img->height+1, img->width+1, CV_32SC1 );
	else
		sum = cvCreateMat( img->height+1, img->width+1, CV_64FC1 );
    cvIntegral( img, sum );

    /* Wavelet Data */
    const int NX=3, NY=3, NXY=4;//, NM=1;
    const int dx_s[NX][5] = { {0, 2, 3, 7, 1}, {3, 2, 6, 7, -2}, {6, 2, 9, 7, 1} };
    const int dy_s[NY][5] = { {2, 0, 7, 3, 1}, {2, 3, 7, 6, -2}, {2, 6, 7, 9, 1} };
    const int dxy_s[NXY][5] = { {1, 1, 4, 4, 1}, {5, 1, 8, 4, -1}, {1, 5, 4, 8, -1}, {5, 5, 8, 8, 1} };
    // const int dm[NM][5] = { {0, 0, 9, 9, 1} };
    CvSurfHF Dx[NX], Dy[NY], Dxy[NXY];//, Dm;

    double dx = 0, dy = 0, dxy = 0;
    int octave, layer, sampleStep, size, margin;
    int rows, cols;
    int i, j, sum_i, sum_j;
    const int* s_ptr;
    float *det_ptr, *trace_ptr;

    /* Allocate enough space for hessian determinant and trace matrices at the 
       first octave. Clearing these initially or between octaves is not
       required, since all values that are accessed are first calculated
    
	CvMat** dets   = (CvMat**) cvStackAlloc( (layers+2) * sizeof(dets[0]) );
    CvMat** traces = (CvMat**) cvStackAlloc( (layers+2) * sizeof(traces[0]) );
    int *sizes     = (int*) cvStackAlloc( (layers+2) * sizeof(sizes[0]) );
    
	for( layer = 0; layer <= layers+1; layer++ )
    {
        dets[layer]   = cvCreateMat( (sum->rows-1)/SAMPLE_STEP0, (sum->cols-1)/SAMPLE_STEP0, CV_32FC1 );
        traces[layer] = cvCreateMat( (sum->rows-1)/SAMPLE_STEP0, (sum->cols-1)/SAMPLE_STEP0, CV_32FC1 );
    }
	--> more efficient implementation within loop!
	*/

    for( octave = 0, sampleStep=SAMPLE_STEP0; octave < octaves; octave++, sampleStep*=2 )
    {
        /* Hessian determinant and trace sample array size in this octave */
        rows = (sum->rows-1)/sampleStep;
        cols = (sum->cols-1)/sampleStep;

		// add new vector for current octave
		detPyr.push_back(   *(new vector<CvMat*>()) );
		tracePyr.push_back( *(new vector<CvMat*>()) );
		//int multipl  = static_cast<int>(pow(2.0f, octave)); 

        /* Calculate the determinant and trace of the hessian */
        for( layer = 0; layer <= layers+1; layer++ )
        {
			// copied from before to create new matrices for each octave
			// dets[layer]   = cvCreateMat( (sum->rows-1)/SAMPLE_STEP0, (sum->cols-1)/SAMPLE_STEP0, CV_32FC1 );
			// traces[layer] = cvCreateMat( (sum->rows-1)/SAMPLE_STEP0, (sum->cols-1)/SAMPLE_STEP0, CV_32FC1 );
			CvMat* det   = cvCreateMat( rows, cols, CV_32FC1 );
			CvMat* trace = cvCreateMat( rows, cols, CV_32FC1 );
			cvSet(det, cvScalar(0.0));
			cvSet(trace, cvScalar(0.0));

			/* Correct implemenation according to original paper from Bay et.al. :
			     size = HAAR_SIZE0 + ( HAAR_SIZE_INC * (multipl-1) ) +  ( HAAR_SIZE_INC * multipl * layer );
			   Implementation in openCV 1.1 pre1a :
		    */
			size = (HAAR_SIZE0+HAAR_SIZE_INC*layer)<<octave;
            icvResizeHaarPattern( dx_s, Dx, NX, 9, size, sum->cols );
            icvResizeHaarPattern( dy_s, Dy, NY, 9, size, sum->cols );
            icvResizeHaarPattern( dxy_s, Dxy, NXY, 9, size, sum->cols );
            // printf( "octave=%d layer=%d size=%d rows=%d cols=%d\n", octave, layer, size, rows, cols );
            
            margin = (size/2)/sampleStep;
            for( sum_i = 0, i = margin;  sum_i <= (sum->rows-1)-size;  sum_i += sampleStep, i++ )
            {
                s_ptr = sum->data.i + sum_i*sum->cols;
                // det_ptr = dets[layer]->data.fl + i*dets[layer]->cols + margin;
				det_ptr = det->data.fl + i*det->cols + margin;
				// trace_ptr = traces[layer]->data.fl + i*traces[layer]->cols + margin;
				trace_ptr = trace->data.fl + i*trace->cols + margin;
                for( sum_j = 0, j = margin;  sum_j <= (sum->cols-1)-size;  sum_j += sampleStep, j++ )
                {
                    dx  = icvCalcHaarPattern( s_ptr, Dx, 3 );
                    dy  = icvCalcHaarPattern( s_ptr, Dy, 3 );
                    dxy = icvCalcHaarPattern( s_ptr, Dxy, 4 );
                    s_ptr += sampleStep;
                    *det_ptr++ = (float)(dx*dy - 0.81*dxy*dxy);
                    *trace_ptr++ = (float)(dx + dy);
                }
            }

			/* add calculated data to pyramid object */
			detPyr.at(octave).push_back( det );
			tracePyr.at(octave).push_back( trace );
        }
	}
}

/** Convert Matrix to IplImage with right scaling of values
 */
void ImagePyramid::convert2DisplayAbleIplImage() {

	// find the absolut minimum and maximum for the dets and traces
	double detMin, detMax;
	double traceMin, traceMax;
	double min, max;
	
	// initialize
	cvMinMaxLoc( detPyr.at(0).at(0), &detMin, &detMax );
	cvMinMaxLoc( tracePyr.at(0).at(0), &traceMin, &traceMax );

	// go over all matrices
	for( unsigned int o = 0; o < detPyr.size(); o++ )
		for( unsigned int i = 0; i < detPyr.at(o).size(); i++ ) {
			cvMinMaxLoc( detPyr.at(o).at(i), &min, &max );
			detMin = (min < detMin) ? min : detMin;
			detMax = (max > detMax) ? max : detMax;
			cvMinMaxLoc( tracePyr.at(o).at(i), &min, &max );
			traceMin = (min < traceMin) ? min : traceMin;
			traceMax = (max > traceMax) ? max : traceMax;
		}

	// define scaling for images
	double detScale = 1/(detMax - detMin);
	double detShift = -detMin * detScale;
	double traceScale = 1/(traceMax - traceMin);
	double traceShift = -traceMin * traceScale;

	// convert the matrices accordingly and create images with this data
	IplImage* detImg, * traceImg;
	CvMat* det, * trace;
	for( unsigned int o = 0; o < detPyr.size(); o++ ) {
		detImgPyr.push_back( *(new vector<IplImage*>()) );
		traceImgPyr.push_back( *(new vector<IplImage*>()) );

		for( unsigned int i = 0; i < detPyr.at(o).size(); i++ ) {
			det = detPyr.at(o).at(i);
			detImg = cvCreateImage(cvSize(det->width, det->height), IPL_DEPTH_32F, 1);
			cvConvertScale(det, detImg, detScale, detShift);
	
			trace = tracePyr.at(o).at(i);
			traceImg = cvCreateImage(cvSize(trace->width, trace->height), IPL_DEPTH_32F, 1);
			cvConvertScale(trace, traceImg, traceScale, traceShift);

			/*
			detImg   = (IplImage*) cvAlloc( sizeof( *detImg ));
			traceImg = (IplImage*) cvAlloc( sizeof( *traceImg ));
			detImg = cvGetImage( detPyr.at(o).at(i), detImg );
			traceImg = cvGetImage( tracePyr.at(o).at(i), traceImg );
			*/

			detImgPyr.at(o).push_back( detImg );
			traceImgPyr.at(o).push_back( traceImg );
		}
	}
}

/** Display the scale space image pyramid
 */
void ImagePyramid::display() {
	if(detPyr.empty() || tracePyr.empty()) {
		cout << "WARNING: Pyramid not built. Nothing to display" << endl;
		return;
	}

	if(detImgPyr.empty() || traceImgPyr.empty()) {
		clearImgPyrs();
		convert2DisplayAbleIplImage();
	}

	for( unsigned int o = 0; o < detPyr.size(); o++ ) {
		for( unsigned int i = 0; i < detPyr.at(o).size(); i++ ) {
			// show determinant
			stringstream dettitle;
			dettitle << "DETERMINANT - octv: " << o << ", intvl: " << i;
			cvNamedWindow(dettitle.str().c_str(), CV_WINDOW_AUTOSIZE); 
			cvShowImage(dettitle.str().c_str(), detImgPyr.at(o).at(i));  
/*			// show trace
			stringstream tracetitle;
			tracetitle << "TRACE - octv: " << o << ", intvl: " << i;
			cvNamedWindow(tracetitle.str().c_str(), CV_WINDOW_AUTOSIZE); 
			cvShowImage(tracetitle.str().c_str(), traceImgPyr.at(o).at(i));  
*/
		}
	}
}

/** Convert the image pyramid to an array of IplImages
 *  Note: Data itself is NOT copied!
 */
IplImage*** ImagePyramid::toArray(char* type) {
	/*
	arr = calloc( octaves, sizeof( IplImage** ) );
	for( int o = 0; o < octaves; o++ )
		arr[o] = calloc( intervals + 2, sizeof(IplImage*) );
	*/
	int t;
	if( !strcmp(type, "det") )
		t = 0;
	else if( !strcmp(type, "trace") )
		t = 1;
	else {
		cout << "Wrong type specified." << endl;
		return NULL;
	}

	IplImage*** arr = new IplImage**[octaves];
	IplImage* img;
	for( int o = 0; o < octaves; o++ ) {
		arr[o] = new IplImage*[intervals+2];

		for( int i = 0; i < intervals+2; i++ ) {
			img = (IplImage*) cvAlloc( sizeof( *img ));
			if( t == 0 )
				cvGetImage( detPyr.at(o).at(i), img );
			else
				cvGetImage( tracePyr.at(o).at(i), img );
			arr[o][i] = img;
		}
	}
	
	return arr;
}