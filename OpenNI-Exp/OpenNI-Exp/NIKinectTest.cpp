#include "NIKinectTest.h"

#include <NIKinect.h>

int main_nikinect_test(int argc, char* argv[]){
	NIKinect* kinect = new NIKinect();

	bool result = false;

	//result = kinect->init_generators();

	cv::Mat color;
	cv::Mat depth;
	cv::Mat mask;
	cv::Mat depthMat8UC1;
	cv::Mat depth_as_color;


	char c = 0;
	while((c = cv::waitKey(31)) != 27){
		if(!kinect->update()) 
			break;

		//if(kinect->get_color(color)){
		//	imshow("Color",color);
		//}

		if(kinect->get_depth(depth)){
			//depth.convertTo(depthMat8UC1, CV_8UC1,0.05);
			//imshow("Depth",depthMat8UC1);
			//
			//kinect->get_mask(mask);
			//imshow("Mask",mask);
		
			kinect->get_depth_as_color(depth_as_color,500,2500);
			cv::imshow("DepthAsColor",depth_as_color);
			//cv::Mat masked_color;
			//color.copyTo(masked_color,mask);
			//imshow("MaskedColor",masked_color);
		}

		printf("Frame Rate: %.2f\n",kinect->get_frame_rate());
	}


	return 0;
}