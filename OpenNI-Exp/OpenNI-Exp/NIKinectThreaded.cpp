#include "NIKinectThreaded.h"
#include <NIKinect.h>

#include <boost\thread.hpp>

int main_nikinect_threaded(int argc, char* argv[]){
	NIKinect* kinect1 = new NIKinect();
	bool result = false;
	char c = 0;
	cv::Mat color_1;
	cv::Mat color_2;
	cv::Mat depth_1;
	cv::Mat depth_2;
	cv::Mat mask_1;
	cv::Mat mask_2;
	cv::Mat depthMat8UC1_1;
	cv::Mat depthMat8UC1_2;

	result = kinect1->init("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni", NIKinect::DEPTH_G + NIKinect::IMAGE_G);

	boost::thread main_thread(&NIKinect::run,kinect1);
	Sleep(199);
	while((c = cv::waitKey(31)) != 27){
	//	if(!kinect1->update()) 
	//		break;

		if(kinect1->get_color(color_1)){
			kinect1->get_mask(mask_1);
			cv::Mat color;

			color_1.copyTo(color,mask_1);
			imshow("Color1",color);
		}

		if(kinect1->get_depth(depth_1)){
			depth_1.convertTo(depthMat8UC1_1, CV_8UC1,0.05);
			cv::imshow("Depth1",depthMat8UC1_1);
		}

		printf("Kinect1 (%.2f)\n",kinect1->get_frame_rate());
	}

	return 0;
}