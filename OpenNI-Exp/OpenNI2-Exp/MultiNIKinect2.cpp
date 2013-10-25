#include "MultiNIKinect2.h"

#include <NIKinect2.h>


int main_multi_nikinect2(int argc, char* argv[]){
	NIKinect2::ni_initialize();

	NIKinect2* kinect = new NIKinect2();

	kinect->initialize();

	kinect->enable_depth_generator();
	kinect->enable_color_generator();

	cv::Mat image_depth, image_color;

	char c = 0;
	while((c = cv::waitKey(11)) != 27){
		NIKinect2::ni_update();

		kinect->update();

		kinect->get_depth_8(image_depth);
		kinect->get_color(image_color);

		cv::imshow("Depth",image_depth);
		cv::imshow("Color",image_color);
	}

	return 0;
}