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

#ifndef COSTESTARG
#define COSTESTARG

#include <iostream>
#include <string>

#include "definitions.h"

using namespace std;


class CovEstArgumentEvaluator {

public:
	/*** Constructor, Destructor ***/
	CovEstArgumentEvaluator() :
		imgDir("./"), imgFile(NULL), keyFileEnding(NULL),
		verbose(true), saveCov(false), showCov(false),
		decType(-1) {};

	CovEstArgumentEvaluator(int argc, char* argv[]) :
		decType(-1) {
		evaluate(argc, argv);
	};

	/*** Methods ***/

	void evaluate(int argc, char* argv[]);


	/*** Member variables ***/
	char* imgDir, * imgFile, * keyFileEnding;
	char imgFileStart[25];
	string keyFile;
	bool verbose, saveCov, showCov;
	int decType;

};


/*** Implementations - quick and dirty ***/


#endif