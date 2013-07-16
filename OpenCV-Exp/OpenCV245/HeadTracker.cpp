#include "HeadTracker.h"

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
Mat frame, curr_frame, gray_frame;
int maxCorners = 23;
//RNG rng(12345);

void detectAndDrawFeatures()
{
  if( maxCorners < 1 ) { maxCorners = 1; }

  /// Parameters for Shi-Tomasi algorithm
  vector<Point2f> corners;// = new vector<Point2f>();
  corners.resize(maxCorners);
  double qualityLevel = 0.01;
  double minDistance = 100;
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  /// Copy the source image
  Mat copy;
  copy = curr_frame.clone();

  /// Apply corner detection
  goodFeaturesToTrack( gray_frame,
	       corners,
	       maxCorners,
	       qualityLevel,
	       minDistance,
	       Mat(),
	       blockSize,
	       useHarrisDetector,
	       k );


  /// Draw corners detected
  int r = 4;
  for( int i = 0; i < corners.size(); i++ )
  {
      std::cout<<corners[i]<<std::endl;
     circle( copy, corners.at(i), r, Scalar(0,0,255),CV_FILLED);
  }


  /// Show what you got
  namedWindow("cenas", CV_WINDOW_AUTOSIZE );
  imshow("cenas", copy );
}


int main_head_tracker(int argc, char* argv[])
{
    VideoCapture capture;

    capture.open(0);
    if (!capture.isOpened())
    {
	printf("Failed to open video device\n");
	return 1;
    }

    for (;;)
    {
	capture>>frame;
	if ( frame.empty() )
	    continue;

	frame.copyTo(curr_frame);
	cvtColor(curr_frame, gray_frame, CV_BGR2GRAY);
	detectAndDrawFeatures();

	//imshow("", curr_frame );

	char key = (char) waitKey(5);

	if( key == ' ' )
	    break;
    }


    while(cvWaitKey(33) != 27)
    {

    }

    return 0;
}