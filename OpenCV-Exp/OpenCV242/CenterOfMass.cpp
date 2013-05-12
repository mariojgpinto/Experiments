#include "CenterOfMass.h"

cv::RNG rng(12345);

int main_center_of_mass(int argc, char* argv[]){
	char* file = "contours.png";

	int thresh = 100;

	//src.convertTo(src_gray,CV_8U);

	while(true){
		cv::Mat src = cv::imread(file);
	cv::Mat src_gray;// = cv::imread(file,0);
	cv::cvtColor(src,src_gray,CV_RGB2GRAY);
		cv::dilate( src_gray, src_gray, cv::Mat() );
		cv::erode( src_gray, src_gray, cv::Mat() );
	//cv::threshold(src,src_gray,0.5,255,CV_THRESH_BINARY);

	cv::Mat1b src_g;
	cv::cvtColor(src,src_g,CV_RGB2GRAY);

	cv::Mat canny_output;
	cv::vector<cv::vector<cv::Point> > contours;
	cv::vector<cv::Vec4i> hierarchy;

	/// Detect edges using canny
	cv::Canny( src_gray, canny_output, thresh, thresh*2, 3 );

	cv::imshow("canny",canny_output);

	/// Find contours
	cv::findContours( canny_output, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
	
	cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );	
	
	int idx = 0;
	int max = -1;
	for(unsigned int i = 0; i< contours.size(); i++ ){
		cv::Scalar color3 = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours( drawing, contours, i, color3, 2, 8, hierarchy, 0, cv::Point() );
		int coiso = contours.at(i).size();
		if(coiso > max){
			idx = i;
			max = coiso;
		}
	}

	cv::Moments mm = moments( contours[idx], false );
	cv::Point2f center = cv::Point2f((float)(mm.m10/mm.m00) , (float)(mm.m01/mm.m00));
	cv::Rect rect = cv::boundingRect(cv::Mat(contours[idx]));


	
	for(unsigned int i = rect.y ; i < rect.y + rect.height; i++){
		
		for(unsigned int j = rect.x ; j < rect.x + rect.width  ; j++){
			cv::Scalar color2 = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			
			if(src_gray.ptr<unsigned char>(i)[j]){
				drawing.ptr<unsigned char>(i)[3*j]	= (unsigned char)color2.val[0]; // first channel
				drawing.ptr<unsigned char>(i)[3*j+1]= (unsigned char)color2.val[1]; // second channel
				drawing.ptr<unsigned char>(i)[3*j+2]= (unsigned char)color2.val[2]; // third channel
				//cv::circle( drawing, pt, 1, color2, -1, 2, 0 );
			}
		}
	}

	cv::Scalar color = cv::Scalar( 255, 255, 255 );
	drawContours( drawing, contours, idx, color, 2, 8, hierarchy, 0, cv::Point() );
	cv::circle( drawing, center, 4, color, -1, 8, 0 );
	cv::rectangle(src,rect,color);

/*	/// Get the moments
	cv::vector<cv::Moments> mu(contours.size() );
	for( int i = 0; i < contours.size(); i++ )
		{ mu[i] = moments( contours[i], false ); }

	///  Get the mass centers:
	cv::vector<cv::Point2f> mc( contours.size() );
	for( int i = 0; i < contours.size(); i++ )
		{ mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }
	

	cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );

	

		cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours( drawing, contours, idx, color, 2, 8, hierarchy, 0, cv::Point() );
		cv::circle( src, mc[idx], 4, color, -1, 8, 0 );*/
	

	
	cv::imshow("src2",src);
	cv::imshow("cont",drawing);

	cv::waitKey(22);
	}

	return 0;
}