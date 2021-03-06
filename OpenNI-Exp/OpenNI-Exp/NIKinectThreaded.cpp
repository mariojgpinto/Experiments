#include "NIKinectThreaded.h"
#include <NIKinect.h>

#include <boost\thread.hpp>

int main_nikinect_threaded(int argc, char* argv[]){
	NIKinect* kinect1 = new NIKinect();
	NIKinect* kinect2 = new NIKinect();
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
	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

	result = kinect1->init("", NIKinect::DEPTH_G + NIKinect::IMAGE_G);
	result = kinect2->init("C:\\Dev\\AAL4ALL\\ONI\\user_calib.oni", NIKinect::DEPTH_G + NIKinect::IMAGE_G);

	boost::thread main_thread(&NIKinect::run,kinect1);
	boost::thread main_thread2(&NIKinect::run,kinect2);
	Sleep(199);
	while((c = cv::waitKey(31)) != 27){
	//	if(!kinect1->update()) 
	//		break;
		cv::Mat color;
		cv::Mat color2;

		kinect1->mutex_lock();
		if(kinect1->get_color(color_1)){
			kinect1->get_mask(mask_1);
			color_1.copyTo(color,mask_1);
			cv::imshow("Color1",color);
		}

		if(kinect1->get_depth(depth_1)){
			depth_1.convertTo(depthMat8UC1_1, CV_8UC1,0.05);
			cv::imshow("Depth1",depthMat8UC1_1);
		}

		kinect1->mutex_unlock();




		kinect2->mutex_lock();
		if(kinect2->get_color(color_2)){
			kinect2->get_mask(mask_2);
			color_2.copyTo(color2,mask_2);
			cv::imshow("Color2",color2);
		}

		if(kinect2->get_depth(depth_2)){
			depth_2.convertTo(depthMat8UC1_2, CV_8UC1,0.05);
			cv::imshow("Depth2",depthMat8UC1_2);
		}

		kinect2->mutex_unlock();


		
		++_frame_counter;
		if (_frame_counter == 15)
		{
			double current_tick = cv::getTickCount();
			_frame_rate = _frame_counter / ((current_tick - _last_tick)/cv::getTickFrequency());
			_last_tick = current_tick;
			_frame_counter = 0;
			
			printf("\nKinect1 (%.2f)\t Kinect2 (%.2f)\twhile (%.2f)\n",	kinect1->get_frame_rate(),
																		kinect2->get_frame_rate(),
																		_frame_rate);
		}
	}

	return 0;
}