#include "MultiNIKinect2ColorAdjust.h"

#include <NIKinect2Manager.h>

int main_multi_nikinect2_color_adjust(int argc, char* argv[]){
	NIKinect2Manager* kinect_manager = new NIKinect2Manager();

	NIKinect2::ni_initialize();

	int n_kinects = kinect_manager->initialize_all_kinects();
	
	int* r = (int*)malloc(sizeof(int) * n_kinects);
	int* g = (int*)malloc(sizeof(int) * n_kinects);
	int* b = (int*)malloc(sizeof(int) * n_kinects);
	int* h = (int*)malloc(sizeof(int) * n_kinects);
	int* s = (int*)malloc(sizeof(int) * n_kinects);
	int* v = (int*)malloc(sizeof(int) * n_kinects);

	for(int i = 0 ; i < n_kinects ; ++i){
		r[i] = 100; g[i] = 100; b[i] = 100;
		h[i] = 100; s[i] = 100; v[i] = 100;

		char win_color[128];
		sprintf(win_color,"Color(%d)",i);
		cv::namedWindow(win_color);
		cv::createTrackbar("R",win_color,&r[i],200);
		cv::createTrackbar("G",win_color,&g[i],200);
		cv::createTrackbar("B",win_color,&b[i],200);
		cv::createTrackbar("H",win_color,&h[i],200);
		cv::createTrackbar("S",win_color,&s[i],200);
		cv::createTrackbar("V",win_color,&v[i],200);

		//NIKinect2* kinect = kinect_manager->get_kinect(i);
		//openni::Status st = kinect->get_color_stream()->getCameraSettings()->setAutoWhiteBalanceEnabled(true);
		//st = kinect->get_color_stream()->getCameraSettings()->setAutoExposureEnabled(true);
		//st = kinect->get_color_stream()->getCameraSettings()->setExposure(50);
		//printf("");
	}

	char c = 0;
	while((c = cv::waitKey(11)) != 27){
		if(!kinect_manager->update_all())
			break;

		for(int k = 0 ; k < n_kinects ; ++k){
			cv::Mat color,color_hsv,color_hsv2,color_trans_rgb,color_trans_hsv;

			NIKinect2* kinect = kinect_manager->get_kinect(k);
			
			if(kinect){
				kinect->get_color(color);

				{//RGB
					cv::vector<cv::Mat> bgr_planes;
					cv::split( color, bgr_planes );

					bgr_planes[0] += b[k] -100;
					bgr_planes[1] += g[k] -100;
					bgr_planes[2] += r[k] -100;

					cv::merge(bgr_planes,color_trans_rgb);
				}

				{//RGB
					cv::cvtColor(color,color_hsv,CV_BGR2HSV);
					cv::vector<cv::Mat> hsv_planes;
					cv::split( color_hsv, hsv_planes );

					hsv_planes[0] += h[k] -100;
					hsv_planes[1] += s[k] -100;
					hsv_planes[2] += v[k] -100;

					cv::merge(hsv_planes,color_hsv2);
					cv::cvtColor(color_hsv2,color_trans_hsv,CV_HSV2BGR);
				}
				
				char win_color[128];
				sprintf(win_color,"Color(%d)",k);
				cv::imshow(win_color,color);

				char win_color_trans_rgb[128];
				sprintf(win_color_trans_rgb,"ColorTransRGB(%d)",k);
				cv::imshow(win_color_trans_rgb,color_trans_rgb);
				
				char win_color_trans_hsv[128];
				sprintf(win_color_trans_hsv,"ColorTransHSV(%d)",k);
				cv::imshow(win_color_trans_hsv,color_trans_hsv);
			}
		}
	} //while

	kinect_manager->~NIKinect2Manager();

	return 0;
}
