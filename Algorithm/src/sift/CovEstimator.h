#pragma once

//c++
#include <iostream>

//opencv
#include <cv.h>
#include <cxcore.h>

//covUtils
#include "definitions.h"

//sift
#include "utils.h"
#include "sift.h"

class CovEstimator 
{

public:
	CovEstimator( const IplImage*** pyr, int type, int octvs, int intvls ) {
		this->type = type;
		octaves = octvs;
		intervals = intvls;
		detecPyr = pyr;
		H = cvCreateMat( 2, 2, CV_32FC1 );
		cov = cvCreateMat( 2, 2, CV_32FC1 );
		evals = cvCreateMat( 2, 1, CV_32FC1 );
		evecs = cvCreateMat( 2, 2, CV_32FC1 );
    }

	~CovEstimator() {
		cleanup();
    }

	const IplImage* getImageFromPyramid(int octv, int intvl);

    CvMat* getCovAt( float x, float y, float scale );

	void drawCovInto( IplImage* img, int x, int y );


private:
	/*** Methods ***/
	CvMat* hessian( const IplImage* img, int r, int c );

	/*  Linear interpolation
		target  - the target point, 0.0 - 1.0
		v       - a pointer to an array of size 2 containg the two values
	*/
	float linearInterp(float target, float v[]) {
		return (float)(target*(v[1])+ (1.0f - target)*(v[0]));
	}

	/*  BiLinear interpolation, linear interpolation in 2D
		target  - a 2D point (X,Y)
		v       - an array of size 4 containg values left to right, top to bottom
		cost: performs 3 linear interpolations
	*/
	float bilinearInterp(float target[], float v[])	{
		float v_prime[2] = { linearInterp(target[0], &(v[0])), 
							 linearInterp(target[0], &(v[2])) }; 
		return linearInterp(target[1], v_prime);
	}

    float trilinearInterp(float target[], float v[])
    {
		float v_prime[2] = { bilinearInterp(&(target[0]), &(v[0])),
							 bilinearInterp(&(target[0]), &(v[4])) };
		return linearInterp(target[2], v_prime);
	}

    void cleanup()
    {
		cvReleaseMat(&H);
		cvReleaseMat(&cov);
		cvReleaseMat(&evals);
		cvReleaseMat(&evecs);
    }


	/*** Member variables ***/
	int type;
	const IplImage*** detecPyr;
	int octaves, intervals;

    CvMat* H;
    CvMat* cov;
    CvMat* evals;
    float ev1, ev2;
    CvMat* evecs;
};
