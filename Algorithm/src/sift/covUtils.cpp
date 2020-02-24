#include <cv.h>
#include <highgui.h>

#include <iostream>
#include <fstream>
#include <time.h>

#include "covUtils.h"
#include <opencv2/imgproc.hpp>

using namespace std;

//-------------------------------------------------------

static const int NCOLOURS = 8;
static const CvScalar COLOURS [] = {cvScalar(255,0,0), cvScalar(0,255,0), 
                                    cvScalar(0,0,255), cvScalar(255,255,0),
                                    cvScalar(0,255,255), cvScalar(255,0,255),
                                    cvScalar(255,255,255), cvScalar(0,0,0)};

//-------------------------------------------------------

//! Display error message and terminate program
void error(char *msg) 
{
  cout << "\nError: " << msg;
  getchar();
  exit(0);
}

//-------------------------------------------------------

//! Show the provided image and wait for keypress
void showImage(IplImage *img)
{
  cvNamedWindow("Surf", CV_WINDOW_AUTOSIZE); 
  cvShowImage("Surf", img);  
  cvWaitKey(0);
}

//-------------------------------------------------------

//! Show the provided image in titled window and wait for keypress
void showImage(const char *title, IplImage *img)
{
  cvNamedWindow(title, CV_WINDOW_AUTOSIZE); 
  cvShowImage(title, img);  
  cvWaitKey(0);
}

//-------------------------------------------------------

// Convert image to single channel 32F
IplImage *getGray(IplImage *img)
{
  // Check we have been supplied a non-null img pointer
  if (!img) error("Unable to create grayscale image.  No image supplied");

  IplImage* gray8, * gray32;

  gray32 = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1 );

  if( img->nChannels == 1 )
    gray8 = (IplImage *) cvClone( img );
  else {
    gray8 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );
    cvCvtColor( img, gray8, CV_BGR2GRAY );
  }

  cvConvertScale( gray8, gray32, 1.0 / 255.0, 0 );

  cvReleaseImage( &gray8 );
  return gray32;
}

//-------------------------------------------------------

//! Draw all the Ipoints in the provided vector
void drawIpoints(IplImage *img, vector<Ipoint> &ipts, int decType, int tailSize) {
	for(unsigned int i = 0; i < ipts.size(); i++) 
		drawIpoint(img, ipts.at(i), decType, tailSize);
	// drawWindows(img, ipts);
}

//-------------------------------------------------------

//! Draw a single feature on the image
void drawIpoint(IplImage *img, Ipoint &ipt, int decType, int tailSize)
{
  	double ori = ipt.orientation;
	int x1 = cvRound(ipt.x);
	int y1 = cvRound(ipt.y);

	// feature point itself
	// cvCircle(img, cvPoint(x1,y1), 2, CV_RGB(255,255,255), 1);
	cvLine( img, cvPoint(x1-1, y1-1), cvPoint(x1+1, y1+1), CV_RGB(0, 0, 0), 2 );
	cvLine( img, cvPoint(x1-1, y1+1), cvPoint(x1+1, y1-1), CV_RGB(0, 0, 0), 2 );
	cvLine( img, cvPoint(x1-1, y1-1), cvPoint(x1+1, y1+1), CV_RGB(0, 0, 255), 1 );
	cvLine( img, cvPoint(x1-1, y1+1), cvPoint(x1+1, y1-1), CV_RGB(0, 0, 255), 1 );

	switch( decType ) {
		case DETECTOR_SIFT: {
			// compute points for an arrow scaled and rotated by feature's scale and orientation
			double scale = 5.0;
			double hscale = 0.75;
			int len  = cvRound( ipt.scale * scale );
			int hlen = cvRound( ipt.scale * hscale );
			int blen = len - hlen;
			int x2 = cvRound(len *  cos(ori)) + x1;
			int y2 = cvRound(len * -sin(ori)) + y1;
			int h1_x = cvRound( blen *  cos(ori + CV_PI/18.0) ) + x1;
			int h1_y = cvRound( blen * -sin(ori + CV_PI/18.0) ) + y1;
			int h2_x = cvRound( blen *  cos(ori - CV_PI/18.0) ) + x1;
			int h2_y = cvRound( blen * -sin(ori - CV_PI/18.0) ) + y1;

			cvLine( img, cvPoint(x1, y1), cvPoint(x2, y2),     CV_RGB(0, 255, 0), 1, 8, 0 );
			cvLine( img, cvPoint(x2, y2), cvPoint(h1_x, h1_y), CV_RGB(0, 255, 0), 1, 8, 0 );
			cvLine( img, cvPoint(x2, y2), cvPoint(h2_x, h2_y), CV_RGB(0, 255, 0), 1, 8, 0 );

			break;
		}

		case DETECTOR_SURF: {
			// points drawn with orientation and circle according to scale of region
			double scale = ((9.0f/1.2) * ipt.scale) / 3.0;
			int lap = ipt.laplacian;

			// Green line indicates orientation
			if (ori) { // Green line indicates orientation
				int x2 = cvRound(scale * cos(ori)) + x1;
				int y2 = cvRound(scale * sin(ori)) + y1;
				cvLine(img, cvPoint(x1, y1), cvPoint(x2, y2), CV_RGB(0, 255, 0));
			} else  // Green dot if using upright version
				cvCircle(img, cvPoint(x1, y1), 1, CV_RGB(0, 255, 0),-1);

			if (lap >= 0) {
				// Blue circles indicate light blobs on dark backgrounds
				cvCircle(img, cvPoint(x1, y1), cvRound(scale), CV_RGB(255, 0, 0),1);
			} else {
				//Red circles indicate light blobs on dark backgrounds
				cvCircle(img, cvPoint(x1, y1), cvRound(scale), CV_RGB(0, 0, 255),1);
			}

			break;
		}
	}

	// Draw motion from ipoint dx and dy
	if (tailSize) {
		cvLine(img, cvPoint(x1,y1),
		cvPoint(int(x1+ipt.dx*tailSize), int(y1+ipt.dy*tailSize)),
		cvScalar(255,255,255), 1);
	}
}

//-------------------------------------------------------

//! Draw a single feature on the image
void drawPoint(IplImage *img, Ipoint &ipt) {
  float s, o;
  int r1, c1;

  s = 3;
  o = ipt.orientation;
  r1 = fRound(ipt.y);
  c1 = fRound(ipt.x);

  cvCircle(img, cvPoint(c1,r1), fRound(s), COLOURS[ipt.clusterIndex%NCOLOURS], -1);
  cvCircle(img, cvPoint(c1,r1), fRound(s+1), COLOURS[(ipt.clusterIndex+1)%NCOLOURS], 2);
}

//-------------------------------------------------------

//! Draw a single feature on the image
void drawPoints(IplImage *img, vector<Ipoint> &ipts) {
  for(unsigned int i = 0; i < ipts.size(); i++) 
	drawPoint(img, ipts[i]);
}

//-------------------------------------------------------

//! Draw descriptor windows around Ipoints in the provided vector
void drawWindows(IplImage *img, vector<Ipoint> &ipts)
{
  Ipoint *ipt;
  float s, o, cd, sd;
  int x, y;
  CvPoint2D32f src[4];

  for(unsigned int i = 0; i < ipts.size(); i++) 
  {
    ipt = &ipts.at(i);
    s = (10 * ipt->scale);
    o = ipt->orientation;
    y = fRound(ipt->y);
    x = fRound(ipt->x);
    cd = cos(o);
    sd = sin(o);

    src[0].x=sd*s+cd*s+x;   src[0].y=-cd*s+sd*s+y;
    src[1].x=sd*s+cd*-s+x;  src[1].y=-cd*s+sd*-s+y;
    src[2].x=sd*-s+cd*-s+x; src[2].y=-cd*-s+sd*-s+y;
    src[3].x=sd*-s+cd*s+x;  src[3].y=-cd*-s+sd*s+y;

    if (o) // Draw orientation line
      cvLine(img, cvPoint(x, y), 
		cvPoint(fRound(s*cd + x), fRound(s*sd + y)), cvScalar(0, 255, 0),1);
    else  // Green dot if using upright version
      cvCircle(img, cvPoint(x,y), 1, cvScalar(0, 255, 0),-1);


    // Draw box window around the point
    cvLine(img, cvPoint(fRound(src[0].x), fRound(src[0].y)), 
      cvPoint(fRound(src[1].x), fRound(src[1].y)), cvScalar(255, 0, 0),1);
    cvLine(img, cvPoint(fRound(src[1].x), fRound(src[1].y)), 
      cvPoint(fRound(src[2].x), fRound(src[2].y)), cvScalar(255, 0, 0),1);
    cvLine(img, cvPoint(fRound(src[2].x), fRound(src[2].y)), 
      cvPoint(fRound(src[3].x), fRound(src[3].y)), cvScalar(255, 0, 0),1);
    cvLine(img, cvPoint(fRound(src[3].x), fRound(src[3].y)), 
      cvPoint(fRound(src[0].x), fRound(src[0].y)), cvScalar(255, 0, 0),1);

  }
}

//-------------------------------------------------------

// Draw the FPS figure on the image (requires at least 2 calls)
void drawFPS(IplImage *img)
{
  static int counter = 0;
  static clock_t t;
  static float fps;
  char fps_text[20];
  CvFont font;
  cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 1.0,1.0,0,2);

  // Add fps figure (every 10 frames)
  if (counter > 10)
  {
    fps = (10.0f/(clock()-t) * CLOCKS_PER_SEC);
    t=clock(); 
    counter = 0;
  }

  // Increment counter
  ++counter;

  // Get the figure as a string
  sprintf(fps_text,"FPS: %.2f",fps);

  // Draw the string on the image
  cvPutText (img,fps_text,cvPoint(10,25), &font, cvScalar(255,255,0));
}

//-------------------------------------------------------

//! Save the features to file
void saveFeatures(const char *filename, vector<Ipoint> &ipts)
{
	if (ipts.size() > 0) {
		ofstream outfile(filename);

		// output descriptor length
		outfile << ipts.at(0).descrLength << "\n";
		outfile << ipts.size() << "\n";

		// create output line as:  scale  x  y  des
		for(unsigned int i=0; i < ipts.size(); i++) {
			outfile << ipts.at(i).scale << "  ";
			outfile << ipts.at(i).x << " ";
			outfile << ipts.at(i).y << " ";
			outfile << ipts.at(i).orientation << " ";
			outfile << ipts.at(i).laplacian << " ";
			// boutfile << ipts.at(i).scale << " ";
			for(int j=0; j<ipts.at(i).descrLength; j++)
				outfile << ipts.at(i).descriptor[j] << " ";

			outfile << "\n";
		}

		outfile.close();
	}
}

//-------------------------------------------------------

//! Load the features from file
void loadFeatures(const char *filename, vector<Ipoint> &ipts)
{
  int descrLength, count;
  ifstream infile(filename);

  // clear the ipts vector first
  ipts.clear();

  // read descriptor length/number of ipoints
  infile >> descrLength;
  infile >> count;

  // for each ipoint
  for (int i = 0; i < count; i++) {
    Ipoint ipt;

    // read vals
    infile >> ipt.scale; 
    infile >> ipt.x;
    infile >> ipt.y;
    infile >> ipt.orientation;
    infile >> ipt.laplacian;
    // infile >> ipt.scale;

    // read descriptor components
	ipt.initializeDescr(descrLength);
    for (int j = 0; j < descrLength; j++)
      infile >> ipt.descriptor[j];

    ipts.push_back(ipt);

  }
}

//-------------------------------------------------------

//-------------------------------------------------------
