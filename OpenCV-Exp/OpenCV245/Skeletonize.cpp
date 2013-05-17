#include "Skeletonize.h"

#include <opencv2\opencv.hpp>

void compute_color_encoded_depth(const cv::Mat& depth_im, cv::Mat& color_depth_im,
                                     double* i_min_val, double* i_max_val)
{
	double min_val, max_val;
	if (i_min_val && i_max_val)
	{
		min_val = *i_min_val;
		max_val = *i_max_val;
	}
	else
	{
		minMaxLoc(depth_im, &min_val, &max_val);
	}

	color_depth_im.create(depth_im.size(),CV_8UC3);
	for (int r = 0; r < depth_im.rows; ++r)
	{
		const float* depth_data = depth_im.ptr<float>(r);
		cv::Vec3b* depth_color_data = color_depth_im.ptr<cv::Vec3b>(r);
		for (int c = 0; c < depth_im.cols; ++c)
		{
			int v = 255*6*(depth_data[c]-min_val)/(max_val-min_val);
			if (v < 0) v = 0;
			char r,g,b;
			int lb = v & 0xff;
			switch (v / 256) {
			case 0:
				r = 255;
				g = 255-lb;
				b = 255-lb;
				break;
			case 1:
				r = 255;
				g = lb;
				b = 0;
				break;
			case 2:
				r = 255-lb;
				g = 255;
				b = 0;
				break;
			case 3:
				r = 0;
				g = 255;
				b = lb;
				break;
			case 4:
				r = 0;
				g = 255-lb;
				b = 255;
				break;
			case 5:
				r = 0;
				g = 0;
				b = 255-lb;
				break;
			default:
				r = 0;
				g = 0;
				b = 0;
				break;
			}
			if (v == 0)
			{
				r = g = b = 0;
			}
			depth_color_data[c] = cv::Vec3b(b,g,r);
		}
	}
}
cv::RNG rng(12345);
int main_skeletonize(int argc, char* argv[]){
	std::vector<cv::Mat> images(3);
	std::vector<cv::Mat> images_bin(3);

	images.at(0) = cv::imread("Walkys\\img_01.png",0);
	images.at(1) = cv::imread("Walkys\\img_02.png",0);
	images.at(2) = cv::imread("Walkys\\img_03.png",0);

	cv::threshold(images.at(0), images_bin.at(0), 1, 255, cv::THRESH_BINARY); 
	cv::threshold(images.at(1), images_bin.at(1), 1, 255, cv::THRESH_BINARY); 
	cv::threshold(images.at(2), images_bin.at(2), 1, 255, cv::THRESH_BINARY); 

	cv::Mat skel(images.at(0).size(), CV_8UC1, cv::Scalar(0));
	cv::Mat temp;
	cv::Mat eroded;
	cv::Mat show;
 
	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(21, 21));

	cv::namedWindow("bin");
	int thresh = 45;
	cv::createTrackbar("thresh","bin",&thresh,255,NULL);

	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

	char c = 0;
	

	double min = 400, max = 1500;
	int im_index = 1;
	while( (c = cv::waitKey(31)) != 27){
		//cv::imshow("Temp",images_bin.at(im_index));
		//cv::waitKey();
		//
		//cv::erode(images_bin.at(im_index), eroded, element);
		//cv::waitKey();

		//cv::dilate(eroded, temp, element); // temp = open(img)
		//cv::imshow("Temp",temp);
		//cv::waitKey();

		//cv::subtract(images_bin.at(im_index), temp, temp);
		//cv::imshow("Temp",temp);
		//cv::waitKey();

		//cv::bitwise_or(skel, temp, skel);
		//cv::imshow("Temp",skel);
		//cv::waitKey();

		//eroded.copyTo(show);
		
		//cv::Mat canny_output;
		//cv::vector<cv::vector<cv::Point> > contours;
		//cv::vector<cv::Vec4i> hierarchy;

		///// Detect edges using canny
		////cv::Canny( images.at(im_index), canny_output, thresh, thresh*2, 3 );
		//cv::findContours( images_bin.at(im_index), contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
	
		//cv::Mat drawing = cv::Mat::zeros( images_bin.at(im_index).size(), CV_8UC3 );	
		//cv::threshold(images_bin.at(im_index), images_bin.at(im_index), 0.1, 255, cv::THRESH_BINARY); 
		//	int idx = 0;
		//int max = -1;
		//for(unsigned int i = 0; i< contours.size(); i++ ){
		//	if(contours[i].size() > 50){
		//		cv::Scalar color3 = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		//		drawContours( drawing, contours, i, color3, -1, 8, hierarchy, 0, cv::Point() );

		//		cv::Rect rect = cv::boundingRect(cv::Mat(contours[i]));
		//		cv::rectangle(drawing,rect,color3);

		//		//Middle line of the contour
		//		//for(int y = rect.y ; y < rect.y + rect.height ; ++y){
		//		//	int left = 0, right = 0;
		//		//	for(int x = rect.x ; !left && x < rect.x + rect.width ; ++x){
		//		//		if(images.at(im_index).ptr<unsigned char>(y)[x])
		//		//			left = x;
		//		//	}

		//		//	for(int x = rect.x + rect.width ; !right && x > left ; --x){
		//		//		if(images.at(im_index).ptr<unsigned char>(y)[x])
		//		//			right = x;
		//		//	}
		//		//
		//		//	int middle = (left+right)/2.0;
		//		//	images.at(im_index).ptr<unsigned char>(y)[middle] = 255;
		//		//	
		//		//	cv::imshow("asfda",images.at(im_index));
		//		//	//cv::waitKey();
		//		//}
		//		
		//		//Higher depth 
		//		//for(int y = rect.y ; y < rect.y + rect.height ; ++y){
		//		//	int min = INT_MAX;
		//		//	int max = -INT_MAX;
		//		//	int idx_max = -1;
		//		//	int idx_min = -1;
		//		//	for(int x = rect.x ; x < rect.x + rect.width ; ++x){
		//		//	
		//		//		int value = images.at(im_index).ptr<unsigned char>(y)[x];

		//		//		if(value&& (value < min)){
		//		//			min = value;
		//		//			idx_min = x;
		//		//		}

		//		//		if(value){
		//		//			images.at(im_index).ptr<unsigned char>(y)[x] = 255;
		//		//		}

		//		//		

		//		//	}
		//		//	images.at(im_index).ptr<unsigned char>(y)[idx_min] = 0;
		//		//	cv::imshow("asfda",images.at(im_index));
		//		//	//cv::waitKey();
		//		//}
		//	}
		//}
		//cv::imshow("canny",drawing);
		cv::Mat distT;
		cv::Mat distT2;
		
		cv::distanceTransform(images_bin.at(im_index),distT,CV_DIST_L2,5);
		//cv::normalize(distT, distT2, 0.0, 1.0, cv::NORM_MINMAX);
		
		//distT2.convertTo(distT3,CV_8UC1);
		cv::imshow("img1",distT);
		//cv::imshow("img2",distT2);
		
		cv::Mat distT3;

		distT.convertTo(distT3,CV_8UC1);

		cv::imshow("8UC1",distT3);
		//cv::Mat co,ga;
		//co = cv::imread("img_q.PNG");

		//int q2 = co.type();

		//cv::cvtColor(co,distT3,CV_RGB2GRAY);

		int q3 = distT3.type();


		cv::Mat distT4;
		int ddepth = CV_32F;
		int asd  = distT2.type();
		cv::Laplacian(distT3,distT4,ddepth,5,1,0,4);
		
		

		double mmin,mmax;
		
		cv::Mat distT5;
		cv::Mat distT6;
		cv::normalize(distT4, distT5, 0, 255, cv::NORM_MINMAX);

		 convertScaleAbs( distT4, distT6 );
		 cv::minMaxIdx(distT6,&mmin, &mmax);


		 cv::Mat distT7;
		 float v = ((float)thresh);
		 cv::threshold(distT6,distT7,v,255,CV_THRESH_BINARY);


		//distT4.convertTo(distT5,CV_32FC1);

		cv::imshow("Laplacian",distT6);
		cv::imshow("Laplacian-4",distT4);
		cv::imshow("bin",distT7);


		cv::Mat distT8;
		cv::dilate(distT7,distT7,cv::Mat(5,5,CV_8UC1));
		cv::erode(distT7,distT7,cv::Mat(3,3,CV_8UC1));

		cv::Mat top_view_color = cv::Mat::zeros(distT7.size(), CV_8UC1);

		cv::vector<cv::vector<cv::Point> > contours;
		cv::vector<cv::Vec4i> hierarchy;
			
		cv::findContours( distT7, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

		for(unsigned int i = 0; i< contours.size(); i++ ){
			if(contours[i].size() > 125){
				cv::Scalar clr = cv::Scalar(255);// rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
				cv::Rect rect = cv::boundingRect(contours[i]);
				drawContours( top_view_color, contours, i, clr, -1, 8, hierarchy, 0, cv::Point() );
				//cv::rectangle(top_view_color,rect, clr);
			}
		}

		cv::imshow("Contours",top_view_color);

		if(c == ' '){ 
			cv::imwrite("ocio.png",distT4);
		}
		cv::Mat bin;
		
		cv::threshold(distT2,bin,v,255,CV_THRESH_BINARY);
		//cv::imshow("bin",bin);
		//cv::waitKey();
		//
		//cv::imshow("img",distT);

		cv::imshow("image",images.at(im_index) + top_view_color);
		cv::imshow("image_bin",images_bin.at(im_index));

		//cv::Mat adapt1; images_bin.at(im_index).convertTo(adapt1,CV_8UC1);
		//cv::Mat adapt2;
		//
		//cv::adaptiveThreshold(adapt1,adapt1,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,11,0);
		//cv::imshow("Adaptative",adapt1);


		if(c == 'q'){//if(c == 37){ // Left arrow
			im_index = ((im_index - 1) < 0) ? 2 : im_index - 1;
		}
		if(c == 'w'){//if(c == 39){ // Right arrow
			im_index = (im_index + 1) % 3;
		}
		if(c != -1){
			printf("");
		}

		++_frame_counter;
		if (_frame_counter == 5)
		{
			double current_tick = cv::getTickCount();
			_frame_rate = _frame_counter / ((current_tick - _last_tick)/cv::getTickFrequency());
			_last_tick = current_tick;
			_frame_counter = 0;
			printf("%.2f\n",_frame_rate);
		}
	}

	return 0;
}




/*
	Skeletonization of a binary image.
	Black values (0) mean object and white values (1 = 255) background.
	Source: Parker, J.R. Algorithms for image processing and computer vision.
					New York, NY: John Wiley & Sons, 1997. 417p. pp. 203-218.

	Program adapted to C/OpenCV by Jose Iguelmar Miranda.
	March, 2010.
  Developed under MacOS X. Watch out the #include directives!
	I have also a Java version of this program.
	jose.ig.miranda@gmail.com
*/
 
#include <OpenCV/cv.h>
#include <OpenCV/highgui.h>

#define NORTH 1
#define SOUTH 3

void skeletonize(IplImage *src); 

int main_skeletonize_net (int argc, char * argv[]) {
	IplImage *image = 0, *srcCopy = 0;
	int w, h, i, j, r, g, b;
	CvScalar pixel, pixOut;
	
	//if(argc != 2) {
	//	printf("Usage: skeletonize <image_file>\n");
	//	exit(1);
	//}
	//std::vector<cv::Mat> images(3);
	//std::vector<cv::Mat> images_bin(3);

	//images.at(0) = cv::imread("Walkys\\img_01.png",0);
	//images.at(1) = cv::imread("Walkys\\img_02.png",0);
	//images.at(2) = cv::imread("Walkys\\img_03.png",0);
	image = cvLoadImage("Walkys\\img_02_bw3.png", CV_LOAD_IMAGE_GRAYSCALE);
	//image = cvLoadImage(argv[1], 1);
	if (!image) {
		printf("Can't find %s\n", argv[1]);
		exit(1);
	}

	w = image->width;
	h = image->height;
	//srcCopy = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 3);
	srcCopy = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);

  for (i = 0; i < h; i++) {
		for (j = 0; j < w; j++) { 
			pixel = cvGet2D(image, i, j);
			b = pixel.val[0];
			if (b > 50)
				pixOut.val[0] = 255;
			else
				pixOut.val[0] = 0;
			cvSet2D(srcCopy, i, j, pixOut);
		}
	}
  cvNamedWindow("Temo", CV_WINDOW_AUTOSIZE);
  cvShowImage("Temp", image);
  cv::waitKey();

	skeletonize(srcCopy);

	cvNamedWindow("Original", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("Original", 100, 100);
	cvShowImage("Original", image);

	cvNamedWindow("Skeleton", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("Skeleton", 150, 150);
	cvShowImage("Skeleton", srcCopy);

	cvWaitKey(0);

	// Release images' buffers...
	cvReleaseImage(&image);
	cvReleaseImage(&srcCopy);
	
	//...and windows
	cvDestroyWindow("Original");
	cvDestroyWindow("Skeleton");

  return 0;
}

// 1-neighbors of pixel.
int nays8(IplImage *im, int r, int c) {
	CvScalar pixel;
	int blue, k = 0, i, j;

  for (i = r-1; i <= r+1; i++) 
		for (j = c-1; j <= c+1; j++) 
			if (i != r || c != j) {
				pixel = cvGet2D(im, i, j);
				blue = pixel.val[0];
				if (blue >= 1)
					k++;
			}

	return k;
}

int connectivity(IplImage *im, int r, int c) {
	int N = 0, b1, b2;
	CvScalar pixel;

	pixel = cvGet2D(im, r, c+1);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r-1, c+1);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0) 
		N++;

	pixel = cvGet2D(im, r-1, c+1);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r-1, c);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0)
		N++;

	pixel = cvGet2D(im, r-1, c);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r-1, c-1);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0)
		N++;

	pixel = cvGet2D(im, r-1, c-1);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r, c-1);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0)
		N++;

	pixel = cvGet2D(im, r, c-1);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r+1, c-1);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0)
		N++;

	pixel = cvGet2D(im, r+1, c-1);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r+1, c);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0)
		N++;

	pixel = cvGet2D(im, r+1, c);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r+1, c+1);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0)
		N++;

	pixel = cvGet2D(im, r+1, c+1);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r, c+1);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0)
		N++;

	return N;
}

void deleteCB(IplImage *im, IplImage *tmp) {
	int w, h, blue, i, j;
	CvScalar pixel;

	w = im->width;
	h = im->height;

	for (i = 1; i < h-1; i++)
		for (int j = 1; j < w-1; j++) {
			pixel = cvGet2D(tmp, i, j);
			blue = pixel.val[0];
			if (blue == 1) {
				pixel.val[0] = 0;
				cvSet2D(im, i, j, pixel);
				cvSet2D(tmp, i, j, pixel);
			}
		}
}

void stair(IplImage *im, IplImage *tmp, int dir) {
	int i, j, b1, b2, b3, b4, b5, b6, b7, b8, b9, w, h;
	CvScalar pixel;
	int N, S, E, W, NE, NW, SE, SW, C;

	w = im->width;
	h = im->height;

	if (dir == NORTH)
		for (i = 1; i < h-1; i++)
			for (j = 1; j < w-1; j++) {
				pixel = cvGet2D(im, i-1, j-1);
				b1 = pixel.val[0];
				pixel = cvGet2D(im, i-1, j);
				b2 = pixel.val[0];
				pixel = cvGet2D(im, i-1, j+1);
				b3 = pixel.val[0];
				pixel = cvGet2D(im, i, j-1);
				b4 = pixel.val[0];
				pixel = cvGet2D(im, i, j);
				b5 = pixel.val[0];
				pixel = cvGet2D(im, i, j+1);
				b6 = pixel.val[0];
				pixel = cvGet2D(im, i+1, j-1);
				b7 = pixel.val[0];
				pixel = cvGet2D(im, i+1, j);
				b8 = pixel.val[0];
				pixel = cvGet2D(im, i+1, j+1);
				b9 = pixel.val[0];
				if (b1 == 1)
					NW = 1;
				else
					NW = 0;
				if (b2 == 1)
					N = 1;
				else
					N = 0;
				if (b3 == 1)
					NE = 1;
				else
					NE = 0;
				if (b4 == 1)
					W = 1;
				else
					W = 0;
				if (b5 == 1)
					C = 1;
				else
					C = 0;
				if (b6 == 1)
					E = 1;
				else
					E = 0;
				if (b7 == 1)
					SW = 1;
				else
					SW = 0;
				if (b8 == 1)
					S = 1;
				else
					S = 0;
				if (b9 == 1)
					SE = 1;
				else
					SE = 0;

				if (dir == NORTH) {
					if (C && !(N && ((E && !NE && !SW && (!W || !S)) || 
						 (W && !NW && !SE && (!E || !S))))) {
						pixel.val[0] = 0;
						cvSet2D(tmp, i, j, pixel);
					} else {
						pixel.val[0] = 1;
						cvSet2D(tmp, i, j, pixel);
					}
				} else if (dir == SOUTH) {
					if (C && !(S && ((E && !SE && !NW && (!W || !N)) || 
						 (W && !SW && !NE && (!E || !N))))) {
						pixel.val[0] = 0;
						cvSet2D(tmp, i, j, pixel);
					} else {
						pixel.val[0] = 1;
						cvSet2D(tmp, i, j, pixel);
					}
				}
			}
}

// Zhang-Suen algorithm.
void skeletonize(IplImage *im) {
	int janelaAH[][2] = {
		{1, 0}, {0, -1}, {-1, 0}, {0, 1}
	};
	int janelaH[][2] = {
		{0, -1}, {1, 0}, {0, 1}, {-1, 0}
	};
	int aBlue[6];
	int w, h, i, v, j, k, blue, lin, col, iJanela, again = 1;
	CvScalar pixel, pixOut;	
	IplImage *tmp = 0;
	
	w = im->width;
	h = im->height;
	tmp = cvCreateImage(cvGetSize(im), IPL_DEPTH_8U, 1);
	
  for (i = 0; i < h; i++) {
		for (j = 0; j < w; j++) { 
			pixel = cvGet2D(im, i, j);
			blue = pixel.val[0];
			if (blue > 0)
				pixel.val[0] = 0;
			else
				pixel.val[0] = 1;
			cvSet2D(im, i, j, pixel);
			pixOut.val[0] = 0;
			cvSet2D(tmp, i, j, pixOut);
		}
	}

	while (again) {
		again = 0;
  	for (i = 1; i < h-1; i++) 
			for (j = 1; j < w-1; j++) { 
				pixel = cvGet2D(im, i, j);
				blue = pixel.val[0];
				if (blue != 1)
					continue;
				k = nays8(im, i, j);
				iJanela = 0;
				if ((k >= 2 && k <= 6) && connectivity(im, i, j) == 1) {
					for (v = 0; v < 6; v++) {
						col = j + janelaAH[iJanela][0];
						lin = i + janelaAH[iJanela][1];
						pixel = cvGet2D(im, lin, col);
						aBlue[v] = pixel.val[0];
						iJanela++;
						if (v == 2) 
							iJanela = 1;
					}
					if (aBlue[0]*aBlue[1]*aBlue[2] == 0 &&
							aBlue[3]*aBlue[4]*aBlue[5] == 0) {
						pixOut.val[0] = 1;
						cvSet2D(tmp, i, j, pixOut);
						again = 1;
					}
				}		// if ((k >= 2...
			}		// for (j = 1;...

			deleteCB(im, tmp);
			if (!again)
				break;

  	for (i = 1; i < h-1; i++) 
			for (j = 1; j < w-1; j++) { 
				pixel = cvGet2D(im, i, j);
				blue = pixel.val[0];
				if (blue != 1)
					continue;
				k = nays8(im, i, j);
				iJanela = 0;
				if ((k >= 2 && k <= 6) && connectivity(im, i, j) == 1) {
					for (v = 0; v < 6; v++) {
						col = j + janelaH[iJanela][0];
						lin = i + janelaH[iJanela][1];
						pixel = cvGet2D(im, lin, col);
						aBlue[v] = pixel.val[0];
						iJanela++;
						if (v == 2) 
							iJanela = 1;
					}
					if (aBlue[0]*aBlue[1]*aBlue[2] == 0 &&
							aBlue[3]*aBlue[4]*aBlue[5] == 0) {
						pixOut.val[0] = 1;
						cvSet2D(tmp, i, j, pixOut);
						again = 1;
					}
				}		// if ((k >= 2...
			}		// for (j = 1;...

		deleteCB(im, tmp);
	}		// while

	stair(im, tmp, NORTH);
	deleteCB(im, tmp);
	stair(im, tmp, SOUTH);
	deleteCB(im, tmp);

  for (i = 1; i < h-1; i++) 
		for (j = 1; j < w-1; j++) { 
			pixel = cvGet2D(im, i, j);
			blue = pixel.val[0];
			if (blue > 0)
				pixel.val[0] = 0;
			else
				pixel.val[0] = 255;
			cvSet2D(im, i, j, pixel);
		}
}		// End skeletonize