#include "VideoRecorder.h"

#include <opencv2\opencv.hpp>

void setup_windows(){
	cv::namedWindow("win", CV_WINDOW_AUTOSIZE);
}

int main_video_recorder(int argc, char* argv[]){
	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;
	
	cv::VideoCapture capture;

	//capture.open(-1);
	//capture.open("C:\\Users\\Pinto\\Videos\\Music\\Postcard_From_1952_Explosions_in_The_Sky_on_98197786(1).mp4");

    if( !capture.isOpened() ){
		return -1;
    }

	//cv::VideoWriter record;

	////std::string::size_type pAt = source.find_last_of('.');                  // Find extension point
	//const std::string NAME = "result_webcam.avi";//source.substr(0, pAt) + argv[2][0] + ".avi";   // Form the new name with container
	//int ex = static_cast<int>(capture.get(CV_CAP_PROP_FOURCC));     // Get Codec Type- Int form

	//// Transform from int to char via Bitwise operators
	//char EXT[] = {(char)(ex & 0XFF) , (char)((ex & 0XFF00) >> 8),(char)((ex & 0XFF0000) >> 16),(char)((ex & 0XFF000000) >> 24), 0};

	//cv::Size S = cv::Size(	(int) capture.get(CV_CAP_PROP_FRAME_WIDTH),    // Acquire input size
	//						(int) capture.get(CV_CAP_PROP_FRAME_HEIGHT));

	//

	//record.open(NAME, ex=-1, capture.get(CV_CAP_PROP_FPS), S, true);

	cv::VideoWriter record("result_webcam.avi", CV_FOURCC('D','I','V','X'), 10, cv::Size(1920,1080), true);
	//cv::VideoWriter record("Result.avi", -1, 30, cv::Size(270,480), true);

	char c = 0;

	cv::Mat image_camera;

	setup_windows();

	while((c = cv::waitKey(11)) != 27){
		capture >> image_camera;
		if( image_camera.empty() )
            break;

		record << image_camera;

		cv::imshow("win",image_camera);

		++_frame_counter;
		if (_frame_counter == 15)
		{
			double current_tick = cv::getTickCount();
			_frame_rate = _frame_counter / ((current_tick - _last_tick)/cv::getTickFrequency());
			_last_tick = current_tick;
			_frame_counter = 0;
			printf("FrameRate %.2f\n",_frame_rate);
		}
	}

	return 0;
}