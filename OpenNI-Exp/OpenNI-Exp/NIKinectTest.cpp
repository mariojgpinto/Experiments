#include "NIKinectTest.h"

#include <NIKinect.h>

int main_nikinect_test(int argc, char* argv[]){
	NIKinect* kinect = new NIKinect();
	bool result = false;
	result = kinect->init("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni");

	//result = kinect->init_generators();

	cv::Mat color;
	cv::Mat depth;
	cv::Mat mask;
	cv::Mat depthMat8UC1;
	cv::Mat3b depth_as_color;

	int _min_bar = 400;
	int _max_bar = 1500;

	cv::namedWindow("DepthAsColor");
	cv::createTrackbar("MinDepth", "DepthAsColor", &_min_bar, 5000, NULL);
	cv::createTrackbar("MaxDepth", "DepthAsColor", &_max_bar, 5000, NULL);

	char c = 0;
	while((c = cv::waitKey(31)) != 27){
		if(!kinect->update()) 
			break;

		kinect->set_min_depth(_min_bar);
		kinect->set_max_depth(_max_bar);
		if(kinect->get_color(color)){
			kinect->get_mask(mask);
			cv::Mat color2;

			color.copyTo(color2,mask);
			imshow("mask",mask);	
			imshow("Color",color2);
		}

		if(kinect->get_depth(depth)){
			//depth.convertTo(depthMat8UC1, CV_8UC1,0.05);
			//imshow("Depth",depthMat8UC1);
			//
			//kinect->get_mask(mask);
			//imshow("Mask",mask);
		
			kinect->get_depth_as_color(depth_as_color);
			cv::imshow("DepthAsColor",depth_as_color);
			//cv::Mat masked_color;
			//color.copyTo(masked_color,mask);
			//imshow("MaskedColor",masked_color);
		}

		printf("Frame Rate: %.2f\n",kinect->get_frame_rate());
	}


	return 0;
}