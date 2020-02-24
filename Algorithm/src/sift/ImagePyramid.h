#pragma once

#include <vector>
#include <iostream>
#include <sstream>

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

// declaration of used internal openCV functions
// #include "cvsurf.h"

using namespace std;

class ImagePyramid
{
public:

	// constructor
	ImagePyramid() {};

	ImagePyramid(IplImage* baseImg, int octaves, int intervals) {
		ImagePyramid();
		this->create(baseImg, octaves, intervals);
	};

	// destructor
	~ImagePyramid() {
		clear();
	};

	// create pyramid
	void create(IplImage* baseImg, int octaves, int intervals);

	// convert image pyramid to array containing IplImages
	IplImage*** toArray(char* type = "det");

	void display();

	// clear all data
	void clear() {
		clearImgPyrs();
		clearMatPyrs();
	}

private:

	/*** Methods ***/
	// convert CvMat to IplImage
	void convert2DisplayAbleIplImage();
	
	// delete pyramids
	void clearMatPyrs() {
		for(unsigned int i = 0; i < detPyr.size(); i++ ) {
			for(unsigned int j = 0; j < detPyr.at(i).size(); j ++ ) {
				cvReleaseMat(&detPyr.at(i).at(j));
				cvReleaseMat(&tracePyr.at(i).at(j));
			}
			detPyr.at(i).clear();
			tracePyr.at(i).clear();
		}
		detPyr.clear();
		tracePyr.clear();
	};

	void clearImgPyrs() {
		for(unsigned int i = 0; i < detImgPyr.size(); i++ ) {
			for(unsigned int j = 0; j < detImgPyr.at(i).size(); j ++ ) {
				cvReleaseImageHeader(&detImgPyr.at(i).at(j));
				cvReleaseImageHeader(&traceImgPyr.at(i).at(j));
			}
			detImgPyr.at(i).clear();
			traceImgPyr.at(i).clear();
		}
		detImgPyr.clear();
		traceImgPyr.clear();
	};

	
	/*** Member variables ***/
	vector< vector< CvMat* > > detPyr;
	vector< vector< CvMat* > > tracePyr;

	vector< vector< IplImage* > > detImgPyr;
	vector< vector< IplImage* > > traceImgPyr;

	int octaves;
	int intervals;

};



