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

#ifndef COVUTILS_H
#define COVUTILS_H

#include <cv.h>
#include "ipoint.h"
#include "definitions.h"
#include <vector>


//! Display error message and terminate program
void error(char *msg);

//! Show the provided image and wait for keypress
void showImage(IplImage *img);

//! Show the provided image in titled window and wait for keypress
void showImage(const char *title, IplImage *img);

// Convert image to single channel 32F
IplImage* getGray(IplImage *img);

//! Draw a single feature on the image
void drawIpoint(IplImage *img, Ipoint &ipt, int decType, int tailSize = 0);

//! Draw all the Ipoints in the provided vector
void drawIpoints(IplImage *img, std::vector<Ipoint> &ipts, int decType, int tailSize = 0);

//! Draw descriptor windows around Ipoints in the provided vector
void drawWindows(IplImage *img, std::vector<Ipoint> &ipts);

// Draw the FPS figure on the image (requires at least 2 calls)
void drawFPS(IplImage *img);

//! Draw a Point at feature location
void drawPoint(IplImage *img, Ipoint &ipt);

//! Draw a Point at all features
void drawPoints(IplImage *img, std::vector<Ipoint> &ipts);

//! Save the SURF features to file
void saveFeatures(const char *filename, std::vector<Ipoint> &ipts);

//! Load the SURF features from file
void loadFeatures(const char *filename, std::vector<Ipoint> &ipts);

//! Round float to nearest integer
inline int fRound(float flt)
{
  return (int) floor(flt+0.5f);
}

#endif