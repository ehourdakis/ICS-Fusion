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

#include "GeneralFeature.h"

ofstream& GeneralFeature::initializeFile(char* filename, IpVec& ipts) {
	ofstream* outfile = new ofstream(filename);
	
	*outfile << "# format:\n# DESCRIPTOR_TYPE\n# NR_INTERESTPOINTS \t DESCRIPTOR_SIZE" << endl;
	*outfile << "SURF" << endl;
	if(ipts.size() > 0)
		*outfile << ipts.size() << "\t" << ipts.at(0).descrLength << endl;
	else
		*outfile << ipts.size() << endl;
	*outfile << "# format:\n# X Y SCALE ORIENTATION LAPLACIAN COVXX COVXY COVYY\n#\t DESCRIPTOR" << endl;
	
	return *outfile;
}

ofstream& GeneralFeature::initializeFile(char* filename, struct feature* features, int nr_feat) {
	ofstream* outfile = new ofstream(filename);
	
	*outfile << "# format:\n# DESCRIPTOR_TYPE\n# NR_INTERESTPOINTS \t DESCRIPTOR_SIZE" << endl;
	*outfile << "SIFT" << endl;
	if(nr_feat > 0)
		*outfile << nr_feat << "\t" << features[0].d << endl;
	else
		*outfile << nr_feat << endl;
	*outfile << "# format:\n# X Y SCALE ORIENTATION 0(LAPLACIAN) COVXX COVXY COVYY\n#\t DESCRIPTOR" << endl;
	
	return *outfile;
}


ofstream& GeneralFeature::initializeFile(string fn, IpVec& ipts) {
	return initializeFile((char*) fn.c_str(), ipts);
}


ofstream& GeneralFeature::initializeFile(string fn, struct feature* features, int nr_feat) {
	return initializeFile((char*) fn.c_str(), features, nr_feat);
}


void GeneralFeature::closeFile(ofstream& outfile) {
	outfile.close();
}


void GeneralFeature::write(ofstream& outfile, Ipoint& ipt, CvMat* cov) {
	/* File format:
	   DETECTOR_TYPE {SURF, SIFT}
	   NR_POINTS DESCRIPTOR_LENGTH
	   X Y scale orientation covxx covxy covyy descr
	   ...
   */

	// interest point data
	outfile << ipt.x << "\t";
	outfile << ipt.y << "\t";
	outfile << ipt.scale << "\t";
	outfile << ipt.orientation << "\t";
	outfile << ipt.laplacian << "\t";
	// outfile << ipt.octave << "\t";
	// outfile << ipt.layer << "\t";

	// covariance information
	outfile << CV_MAT_ELEM(*cov, float, 0, 0) << "\t";
	outfile << CV_MAT_ELEM(*cov, float, 0, 1) << "\t";
	outfile << CV_MAT_ELEM(*cov, float, 1, 1) << "\t";
	outfile << endl;
	
	// descriptor in new line with initial tab
	if( ipt.descrLength > 0 ) {
		for( int j = 0; j < ipt.descrLength; j++ )
			outfile << "\t" << ipt.descriptor[j] ;

		outfile << endl;
	}
}


void GeneralFeature::write(ofstream& outfile, struct feature& feat, CvMat* cov) {
	/* File format:
	   DETECTOR_TYPE {SURF, SIFT}
	   NR_POINTS DESCRIPTOR_LENGTH
	   X Y scale orientation covxx covxy covyy descr
	   ...
   */

	// interest point data
	outfile << feat.x << "\t";
	outfile << feat.y << "\t";
	outfile << feat.scl << "\t";
	outfile << feat.ori << "\t";
	outfile << 0 << "\t";

	// covariance information
	outfile << CV_MAT_ELEM(*cov, float, 0, 0) << "\t";
	outfile << CV_MAT_ELEM(*cov, float, 0, 1) << "\t";
	outfile << CV_MAT_ELEM(*cov, float, 1, 1) << "\t";
	outfile << endl;
	
	// descriptor in new line with initial tab
	if( feat.d > 0 ) {
		for( int j = 0; j < feat.d; j++ )
			outfile << "\t" << feat.descr[j] ;

		outfile << endl;
	}
}


KeyPoints& GeneralFeature::readFrom(string filename) {

	KeyPoints* keys = new KeyPoints();

	// create input filestream
	ifstream infile( filename.c_str() , ifstream::in );
	if(!infile.good()) {
		cout << "Specified file " << filename << " can not be read." << endl;
		return *(new KeyPoints());
	}

	// read file format
	GeneralFeature::skiptComments(infile);
	string type;
	int nrPoints, descrLen;
	infile >> type >> nrPoints >> descrLen;
	
	// read the point data
	GeneralFeature::skiptComments(infile);
	int i = 0;
	GeneralFeature* gf;
	float x, y, scale, orientation, covxx, covxy, covyy;
	int laplacian;
	
	while(infile.good() && i++ < nrPoints) {
		// new general feature
		gf = new GeneralFeature(type, descrLen);

		// read the position, scale etc
		infile >> x >> y >> scale >> orientation >> laplacian >> covxx >> covxy >> covyy;
		gf->set(x, y, scale, orientation, covxx, covxy, covyy, laplacian);
	
		// read the descriptor if specified
		if(descrLen > 0)
			gf->readAndSaveDescriptor(infile);

		// add to vector of keys
		keys->push_back(gf);
	}

	// return vector read keys
	return *keys;
}

void GeneralFeature::readAndSaveDescriptor(ifstream& infile) {
	for( int idx = 0; idx < this->descrLen; idx++ )
		infile >> this->descriptor[idx];
}