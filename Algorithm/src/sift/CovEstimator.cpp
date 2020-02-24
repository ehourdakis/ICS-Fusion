#include "CovEstimator.h"
#include <opencv2/imgproc.hpp>

using namespace std;

#define PI		3.14159265f


const IplImage* CovEstimator::getImageFromPyramid(int octv, int intvl) {

	/* Interpolation between layers based on the parameter subscale would be possible here,
	   but not implemented.
    */
	return detecPyr[octv][intvl];
}


CvMat* CovEstimator::getCovAt( float x, float y, float scale )
{
    // Retrieve the octave and interval the feature was detected at
    int octv=0;
    int intv=0;
    int row=0;
    int col=0;
    float subintv=0.0f, subscale=0.0f;

    switch(this->type)
    {
        case DETECTOR_SIFT:
        {
                // scale calculation: scl = sigma * 2 ^ ((octv*intlvs + inv)/intvls) * 1/ 2;
                float tmp  = log(2 * scale / SIFT_SIGMA) / log(2.0f) * intervals;
                intv = ((int)cvRound(tmp) - 1) % intervals + 1;
                octv = ((int)cvRound(tmp) - 1) / intervals;
                subintv = tmp - (octv*intervals + intv);
                subscale = scale - SIFT_SIGMA/2 * pow(2.0f, (octv*intervals + intv)/intervals);

                // location calculation: feat->x = ( c + xc ) * pow( 2.0, octv );
                col = cvRound( x / pow(2.0, octv-1) );
                row = cvRound( y / pow(2.0, octv-1) );

                break;
        }
        case DETECTOR_SURF:
        {
                float size = scale * 9.0f/1.2f;
                if( size < 24 )
                    octv = 0;
                else if( size < 48 ) // 45
                    octv = 1;
                else if( size < 96 ) // 87
                    octv = 2;
                else
                    octv = 3;

                float tmp = (size/pow(2.0f, octv) - 9.0f) / 6.0f;
                intv = cvRound(tmp);
                subintv = tmp - intv;
                float subsize = static_cast<float>(size)
                            - (9 + 6*static_cast<float>(intv))*pow(2.0f, octv);
                subscale = 1.2f * subsize / 9.0f;

                col = cvRound( x / pow(2.0, octv) );
                row = cvRound( y / pow(2.0, octv) );

                break;
        }
    }

    // extract right image from pyramid
    const IplImage* img = getImageFromPyramid(octv, intv);

    // determine hessan at that point and calculate and scale covariance matrix
    this->H = hessian( img, row, col );
    cvInvert( this->H, this->cov, CV_SVD_SYM );


    switch(this->type)
    {
        case DETECTOR_SIFT:
            cvScale( cov, cov, pow(2.0f, (octv + subintv/intervals) * 2) * 0.01 );
            break;
        case DETECTOR_SURF:
            cvScale( cov, cov, pow(1.2 * pow(2.0f, octv) + subscale, 2) * 10000);
            break;

    }

    cvSVD(cov,evals, evecs);
    ev1 = CV_MAT_ELEM(*evals, float, 0, 0);
    ev2 = CV_MAT_ELEM(*evals, float, 1, 0);
    if( ev1 < 0 && ev2 < 0 )
    {
        ev1 = -ev1;
        ev2 = -ev2;
    }
    if( ev1 < ev2 )
    {
        float tmp = ev1;
        ev1 = ev2;
        ev2 = tmp;
    }
    if( ev1 <= 0 || ev2 <= 0 )
    {
        cout << "COV Eigenvalue of Hessian is negativ or zero(!)" << endl;
    }

    return cov;
}


void CovEstimator::drawCovInto(IplImage *img, int x, int y)
{
	CvSize axes;
	axes = cvSize(static_cast<int>(ev1),
					static_cast<int>(ev2));
	float vx = CV_MAT_ELEM(*evecs, float, 0, 0);
	float vy = CV_MAT_ELEM(*evecs, float, 1, 0);
	float angle = sin( vy / sqrt(vx*vx + vy*vy) ) * 180/PI;
	cvEllipse( img,
			   cvPoint( x, y ), 
			   axes,
			   angle,
			   0,
			   360,
			   CV_RGB(0, 0, 0), 
			   2 );
	cvEllipse( img,
			   cvPoint( x, y ), 
			   axes,
			   angle,
			   0,
			   360,
			   CV_RGB(0, 255, 0), 
			   1 );
	// plot point
	// cvCircle( img, cvPoint(x,y), 1, CV_RGB(255, 0, 0) );
	cvLine( img, cvPoint(x-1, y-1), cvPoint(x+1, y+1), CV_RGB(0, 0, 0), 2 );
	cvLine( img, cvPoint(x-1, y+1), cvPoint(x+1, y-1), CV_RGB(0, 0, 0), 2 );
	cvLine( img, cvPoint(x-1, y-1), cvPoint(x+1, y+1), CV_RGB(0, 0, 255), 1 );
	cvLine( img, cvPoint(x-1, y+1), cvPoint(x+1, y-1), CV_RGB(0, 0, 255), 1 );
}


CvMat* CovEstimator::hessian( const IplImage* dog, int row, int col ) {

	int r, c;
/*	r = row; c = col;
	float v   =   pixval32f( dog, r, c );
	float dxx = ( pixval32f( dog, r, c+1 ) + 
				   pixval32f( dog, r, c-1 ) - 2 * v );
	float dyy = ( pixval32f( dog, r+1, c ) +
				   pixval32f( dog, r-1, c ) - 2 * v );
	float dxy = ( pixval32f( dog, r+1, c+1 ) -
				   pixval32f( dog, r+1, c-1 ) -
				   pixval32f( dog, r-1, c+1 ) +
				   pixval32f( dog, r-1, c-1 ) ) / 4.0;

*/	// ---
	float v, dxx = 0, dyy = 0, dxy = 0;
	float w[3][3] = { 0.0449f,    0.1221f,    0.0449f,
					  0.1221f,    0.3319f,    0.1221f,
					  0.0449f,    0.1221f,    0.0449f };

	for(int i = 0; i < 3; i++ )
		for( int j = 0; j < 3; j++ ) {
			r = row+(j-1); c = col+(i-1);
			v   =   pixval32f( dog, r, c );
			dxx += w[i][j] * ( pixval32f( dog, r, c+1 ) + 
							   pixval32f( dog, r, c-1 ) - 2 * v );
			dyy += w[i][j] * ( pixval32f( dog, r+1, c ) +
							   pixval32f( dog, r-1, c ) - 2 * v );
			dxy += w[i][j] * ( pixval32f( dog, r+1, c+1 ) -
							   pixval32f( dog, r+1, c-1 ) -
							   pixval32f( dog, r-1, c+1 ) +
							   pixval32f( dog, r-1, c-1 ) ) / 4.0f;

		}

	cvmSet( H, 0, 0, -dxx );
	cvmSet( H, 0, 1, -dxy );
	cvmSet( H, 1, 0, -dxy );
	cvmSet( H, 1, 1, -dyy );

	return H;
}
