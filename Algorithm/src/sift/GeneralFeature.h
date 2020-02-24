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

#pragma once

// openCV
#include <cv.h>

// C++
#include <string>
#include <fstream>
#include <cmath>
#include <iostream>

// own
#include "ipoint.h"

//sift
extern "C" {
	#include "imgfeatures.h"
}


class GeneralFeature;
typedef std::vector<GeneralFeature*>	KeyPoints;

using namespace std;

// template <typename T>
class GeneralFeature {

public:

	GeneralFeature() :
		laplacian(0), descrLen(0), descriptor(NULL) {};

	GeneralFeature(string& type, int dcrLen) {
		this->type = type;
		this->laplacian = 0;
		this->initializeDescr(dcrLen);
	}

	GeneralFeature(GeneralFeature& feat) {
		this->type = feat.type;
		this->set(feat.x, feat.y, feat.scale, feat.orientation, feat.covxx, feat.covxy, feat.covyy, feat.laplacian);
		this->initializeDescr(feat.descrLen);
		if( this->descrLen > 0 ) {
			for( int i = 0; i < descrLen; i++ )
				this->descriptor[i] = feat.descriptor[i];
		}
	}

	~GeneralFeature() {
		deleteDescr();
	}

	void set(float x, float y, float scale, float orientation,
			 float covxx, float covxy, float covyy, int laplacian = 0) {
		this->x = x;
		this->y = y;
		this->scale = scale;
		this->orientation = orientation;
		this->covxx = covxx;
		this->covxy = covxy;
		this->covyy = covyy;
		this->laplacian = laplacian;
	}

	void readAndSaveDescriptor(ifstream& infile) ;	

	static ofstream& initializeFile(char* filename, IpVec& ipts);
	static ofstream& initializeFile(string fn, IpVec& ipts);
	static ofstream& initializeFile(char* filename, struct feature* features, int nr_feat);
	static ofstream& initializeFile(string fn, struct feature* features, int nr_feat);

	static void closeFile(ofstream& outfile);

	static void write(ofstream& outfile, Ipoint& ipt, CvMat* cov);
	static void write(ofstream& outfile, struct feature& feature, CvMat* cov);

	static KeyPoints& readFrom(string filename);

	static void skiptComments(ifstream& infile) {
		string line;
		int position = 0;
		do {
			position = infile.tellg();
			getline( infile, line, '\n' );
		} while( line[0] == '#' || line == "" );
		infile.seekg(position);
	};

	float* getDescr() { return descriptor; };
	int getDescrLen() { return descrLen; };
	int getLaplacian() { return laplacian; };
	CvPoint getCvPoint() { 
		return cvPoint( cvRound(x), cvRound(y) );
	};

private:

	/*** Methods ***/
	void initializeDescr(int len) {
		if( len > 0 ) {
			descriptor = new float[len];
			descrLen = len;
		}
		else {
			descriptor = NULL;
			descrLen = 0;
		}
	}

	void deleteDescr() { 
		if(descriptor) {
			delete [] descriptor;
			descriptor = NULL;
		}
		descrLen = 0;
	}

public:
	/*** Member variables ***/
	string type;
	float x, y, scale, orientation;
	float covxx, covxy, covyy;
	int laplacian;
	int descrLen;
	float* descriptor;


};