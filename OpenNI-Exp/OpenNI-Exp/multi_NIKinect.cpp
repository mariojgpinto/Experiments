#include <_ID.h>

#include "multi_NIKinect.h"
#include <NIKinect.h>





int main_multi_ni_kinect(int argc, char* argv[]){
	NIKinect* kinect1 = new NIKinect();
	NIKinect* kinect2 = new NIKinect();
	bool result = false;

#ifdef _CCG
	result = kinect1->init();
	result = kinect2->init();
#endif

#ifdef _HOME
	result = kinect->init("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni", NIKinect::DEPTH_G + NIKinect::IMAGE_G + NIKinect::SCENE_A + NIKinect::USER_G);
#endif
	

	//kinect->set_processing_flag(NIKinect::DEPTH_COLOR,true);
	//result = kinect->init_generators();

	cv::Mat color_1;
	cv::Mat color_2;
	cv::Mat depth_1;
	cv::Mat depth_2;
	cv::Mat mask_1;
	cv::Mat mask_2;
	cv::Mat depthMat8UC1_1;
	cv::Mat depthMat8UC1_2;
	cv::Mat3b depth_as_color;

	int _min_bar = 400;
	int _max_bar = 1500;

	cv::namedWindow("DepthAsColor");
	cv::createTrackbar("MinDepth", "DepthAsColor", &_min_bar, 5000, NULL);
	cv::createTrackbar("MaxDepth", "DepthAsColor", &_max_bar, 5000, NULL);

	char c = 0;
	while((c = cv::waitKey(31)) != 27){
		if(!kinect1->update()) 
			break;
		if(!kinect2->update()) 
			break;

		//kinect->set_min_depth(_min_bar);
		//kinect->set_max_depth(_max_bar);
		if(kinect1->get_color(color_1)){
			kinect1->get_mask(mask_1);
			cv::Mat color;

			color_1.copyTo(color,mask_1);
			imshow("Color1",color);
		}
		if(kinect2->get_color(color_2)){
			kinect2->get_mask(mask_2);
			cv::Mat color;

			color_2.copyTo(color,mask_2);
			imshow("Color2",color);
		}

		if(kinect1->get_depth(depth_1)){
			depth_1.convertTo(depthMat8UC1_1, CV_8UC1,0.05);
			imshow("Depth1",depthMat8UC1_1);
			//
			//kinect->get_mask(mask);
			//imshow("Mask",mask);
		
			//if(kinect->get_depth_as_color(depth_as_color)){
			//	cv::imshow("DepthAsColor",depth_as_color);
			//}
			//cv::Mat masked_color;
			//color.copyTo(masked_color,mask);
			//imshow("MaskedColor",masked_color);
		}

		if(kinect2->get_depth(depth_2)){
			depth_2.convertTo(depthMat8UC1_2, CV_8UC1,0.05);
			imshow("Depth2",depthMat8UC1_2);
		}

		printf("Kinect1 (%.2f)    Kinect2 (%.2f)\n",kinect1->get_frame_rate(), kinect2->get_frame_rate());
	}


	return 0;
}