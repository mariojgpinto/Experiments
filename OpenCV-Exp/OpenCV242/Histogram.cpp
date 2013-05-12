#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

typedef struct RgbColor
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
} RgbColor;

typedef struct HsvColor
{
    unsigned char h;
    unsigned char s;
    unsigned char v;
} HsvColor;

RgbColor HsvToRgb(HsvColor hsv)
{
    RgbColor rgb;
    unsigned char region, remainder, p, q, t;

    if (hsv.s == 0)
    {
        rgb.r = hsv.v;
        rgb.g = hsv.v;
        rgb.b = hsv.v;
        return rgb;
    }

    region = hsv.h / 43;
    remainder = (hsv.h - (region * 43)) * 6; 

    p = (hsv.v * (255 - hsv.s)) >> 8;
    q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
    t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
        case 0:
            rgb.r = hsv.v; rgb.g = t; rgb.b = p;
            break;
        case 1:
            rgb.r = q; rgb.g = hsv.v; rgb.b = p;
            break;
        case 2:
            rgb.r = p; rgb.g = hsv.v; rgb.b = t;
            break;
        case 3:
            rgb.r = p; rgb.g = q; rgb.b = hsv.v;
            break;
        case 4:
            rgb.r = t; rgb.g = p; rgb.b = hsv.v;
            break;
        default:
            rgb.r = hsv.v; rgb.g = p; rgb.b = q;
            break;
    }

    return rgb;
}

HsvColor RgbToHsv(RgbColor rgb)
{
    HsvColor hsv;
    unsigned char rgbMin, rgbMax;

    rgbMin = rgb.r < rgb.g ? (rgb.r < rgb.b ? rgb.r : rgb.b) : (rgb.g < rgb.b ? rgb.g : rgb.b);
    rgbMax = rgb.r > rgb.g ? (rgb.r > rgb.b ? rgb.r : rgb.b) : (rgb.g > rgb.b ? rgb.g : rgb.b);

    hsv.v = rgbMax;
    if (hsv.v == 0)
    {
        hsv.h = 0;
        hsv.s = 0;
        return hsv;
    }

    hsv.s = 255 * long(rgbMax - rgbMin) / hsv.v;
    if (hsv.s == 0)
    {
        hsv.h = 0;
        return hsv;
    }

    if (rgbMax == rgb.r)
        hsv.h = 0 + 43 * (rgb.g - rgb.b) / (rgbMax - rgbMin);
    else if (rgbMax == rgb.g)
        hsv.h = 85 + 43 * (rgb.b - rgb.r) / (rgbMax - rgbMin);
    else
        hsv.h = 171 + 43 * (rgb.r - rgb.g) / (rgbMax - rgbMin);

    return hsv;
}

/// Function Headers
void Hist_and_Backproj(Mat src, Mat hsv, Mat hue, char* image);

void show_image(char* image);

//@function main *
int main( int argc, char** argv )
{
	show_image("dn2.jpg");
	show_image("MyMap.bmp");
	cv::waitKey(0);

	return ( 0);
}

void show_image(char* image){
	if(!image) return ;

	Mat src; Mat hsv; Mat hue;

	/// Read the image
	src = imread( image, 1 );
	/// Transform it to HSV
	cvtColor( src, hsv, CV_BGR2HSV );

	/// Use only the Hue value
	hue.create( hsv.size(), hsv.depth() );
	int ch[] = { 0, 0 };
	mixChannels( &hsv, 1, &hue, 1, ch, 1 );

	Hist_and_Backproj(src, hsv, hue,image);
}

//@function Hist_and_Backproj
//@brief Callback to Trackbar

void Hist_and_Backproj(Mat src, Mat hsv, Mat hue, char* image)
{
	std::string window_image("Source ");
	window_image.append(image);
	std::string window_hist("Histogram ");
	window_hist.append(image);

	int bins = 255;

	MatND hist;
	int histSize = MAX( bins, 2 );
	float hue_range[] = { 0, 180 };
	const float* ranges = { hue_range };
  
	Mat h_hist, s_hist, v_hist;

	const float* histRange = { hue_range };

	bool uniform = true; bool accumulate = false;

	// Draw the histograms for B, G and R
	int hist_w = 400; int hist_h = 400;
	int bin_w = cvRound( (double) hist_w/histSize );

	Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

	/// Get the Histogram and normalize it
	cv::calcHist( &hue, 1, 0, Mat(), hist, 1, &histSize, &ranges, true, false );
	cv::normalize( hist, hist, 0, 255, NORM_MINMAX, -1, Mat() );

	/// Get Backprojection
	MatND backproj;
	calcBackProject( &hue, 1, 0, hist, backproj, &ranges, 1, true );

	/// Draw the histogram
	int w = 400; int h = 400;
	int bin_w2 = cvRound( (double) w / histSize );
	Mat histImg = Mat::zeros( w, h, CV_8UC3 );
  
	for( int i = 0; i < bins; i ++ ){ 
		HsvColor coiso_hsv;
		coiso_hsv.h = i;
		coiso_hsv.s = 255;
		coiso_hsv.v = 255;

		RgbColor coiso_rgb = HsvToRgb(coiso_hsv);

		rectangle( histImg, Point( i*bin_w2, h ), Point( (i+1)*bin_w2, h - cvRound( hist.at<float>(i)*h/255.0 ) ), Scalar( coiso_rgb.b, coiso_rgb.g , coiso_rgb.r), -1 ); //
	}

	cv::imshow( window_hist.data(), histImg );
	cv::imshow( window_image.data(), src );

	cv::waitKey(10);
}//*/

/*
 //@function main
 
int main( int argc, char** argv )
{
  Mat src, dst;
  Mat hsv;
  /// Load image
  src = imread( "dn2.jpg", 1 );

  if( !src.data )
    { return -1; }

  cvtColor( src, hsv, CV_BGR2HSV );
  /// Separate the image in 3 places ( B, G and R )
  vector<Mat> bgr_planes;
  split( hsv, bgr_planes );

  /// Establish the number of bins
  int histSize = 256;

  /// Set the ranges ( for B,G,R) )
  float range[] = { 0, 256 } ;
  const float* histRange = { range };

  bool uniform = true; bool accumulate = false;

  Mat b_hist, g_hist, r_hist;

  /// Compute the histograms:
  calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
  calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
  calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

  // Draw the histograms for B, G and R
  int hist_w = 512; int hist_h = 400;
  int bin_w = cvRound( (double) hist_w/histSize );

  Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

  /// Normalize the result to [ 0, histImage.rows ]
  normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
  normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
  normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

  /// Draw for each channel
  for( int i = 1; i < histSize; i++ )
  {
      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                       Scalar( 255, 0, 0), 2, 8, 0  );
      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                       Scalar( 0, 255, 0), 2, 8, 0  );
      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                       Scalar( 0, 0, 255), 2, 8, 0  );
  }

  /// Display
  //namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
  imshow("calcHist Demo", histImage );
  imshow("orig",src);
  waitKey(0);

  return 0;
}//*/

/*
#include <opencv2\opencv.hpp>


using namespace cv;

int main( int argc, char** argv )
{
    Mat src;
    if(!(src=imread("dn2.jpg", 1)).data )
        return -1;

    Mat hsv;
    cvtColor(src, hsv, CV_BGR2HSV);

    // let's quantize the hue to 30 levels
    // and the saturation to 32 levels
    int hbins = 30, sbins = 32;
    int histSize[] = {hbins, sbins};
    // hue varies from 0 to 179, see cvtColor
    float hranges[] = { 0, 180 };
    // saturation varies from 0 (black-gray-white) to
    // 255 (pure spectrum color)
    float sranges[] = { 0, 256 };
    const float* ranges[] = { hranges, sranges };
    MatND hist;
    // we compute the histogram from the 0-th and 1-st channels
    int channels[] = {0, 1};

    calcHist( &hsv, 1, channels, Mat(), // do not use mask
        hist, 2, histSize, ranges,
        true, // the histogram is uniform
        false );
    double maxVal=0;
    minMaxLoc(hist, 0, &maxVal, 0, 0);

    int scale = 10;
    cv::Mat histImg = Mat::zeros(sbins*scale, hbins*10, CV_8UC3);

    //for( int h = 0; h < hbins; h++ )
    //    for( int s = 0; s < sbins; s++ )
    //    {
    //        float binVal = hist.at<float>(h, s);
    //        int intensity = cvRound(binVal*255/maxVal);
    //        cv::rectangle( histImg, Point(h*scale, s*scale),
    //                     Point( (h+1)*scale - 1, (s+1)*scale - 1),
    //                     Scalar::all(intensity),
    //                     CV_FILLED );
    //    }

	
	int w = 400; int h = 400;
	int bin_w = cvRound( (double) w / hbins );
	//Mat histImg = Mat::zeros( w, h, CV_8UC3 );

	for( int i = 0; i < hbins; i ++ )
	   { rectangle( histImg, Point( i*bin_w, h ), Point( (i+1)*bin_w, h - cvRound( hist.at<float>(i)*h/255.0 ) ), Scalar( 0, 0, 255 ), -1 ); }

    namedWindow( "Source", 1 );
    imshow( "Source", src );

    namedWindow( "H-S Histogram", 1 );
    imshow( "H-S Histogram", histImg );

    waitKey(0);
}

*/