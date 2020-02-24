// Modified version of:

/*********************************************************** 
*  --- OpenSURF ---                                        *
*  This library is distributed under the GNU GPL. Please   *
*  contact chris.evans@irisys.co.uk for more information.  *
*                                                          *
*  C. Evans, Research Into Robust Visual Features,         *
*  MSc University of Bristol, 2008.                        *
*                                                          *
************************************************************/

#ifndef IPOINT_H
#define IPOINT_H

#include <vector>
#include <math.h>

//-------------------------------------------------------

class Ipoint; // Pre-declaration
typedef std::vector<Ipoint> IpVec;
typedef std::vector<std::pair<Ipoint, Ipoint> > IpPairVec;

//-------------------------------------------------------

//! Ipoint operations
void getMatches(IpVec &ipts1, IpVec &ipts2, IpPairVec &matches);
int translateCorners(IpPairVec &matches, const CvPoint src_corners[4], CvPoint dst_corners[4]);

//-------------------------------------------------------

class Ipoint {

public:

	//! Destructor
    ~Ipoint()
    {
        if(cov!=nullptr)
        {
            //cvReleaseMat(&cov);
            cov=nullptr;
        }
    }

	//! Constructor
	Ipoint() : x(0.0), y(0.0), scale(0.0), orientation(0.0), laplacian(0),
			   descriptor(NULL), descrLength(0),
			   dx(0.0), dy(0.0), clusterIndex(0),
               octave(-1), layer(-1),cov(nullptr){};

	//! Gets the distance in descriptor space between Ipoints
    float operator-(const Ipoint &rhs)
    {
		float sum=0.f;
		for(int i=0; i < descrLength; i++)
			sum += (this->descriptor[i] - rhs.descriptor[i])*(this->descriptor[i] - rhs.descriptor[i]);
		return sqrt(sum);
	};

	void initializeDescr(int size) {
		descrLength = size;
		descriptor = new float[size];
	};

	//! Coordinates of the detected interest point
	float x, y;
	//! Detected scale
	float scale;
	//! Orientation measured anti-clockwise from +ve x-axis
	float orientation;
	//! Sign of laplacian for fast matching purposes
	int laplacian;
	//! Vector of descriptor components
	float* descriptor;
	//! Length of descriptor
	int descrLength;
	//! Placeholds for point motion (can be used for frame to frame motion analysis)
	float dx, dy;
	//! Used to store cluster index
	int clusterIndex;
	//! Octave and layer the point was detected at
	int octave, layer;
	
    CvMat *cov;
};

//-------------------------------------------------------


#endif
