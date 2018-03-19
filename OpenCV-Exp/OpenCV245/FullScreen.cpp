#include "FullScreen.h"

#include <opencv2\opencv.hpp>

int main_full_screen(int argc, char* argv[]){
//	cv::setUseOptimized(true);

	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;


	cv::VideoCapture capture;

	capture.open(1);
	//capture.open("C:\\Users\\Pinto\\Videos\\Music\\Radiohead - Lotus Flower.mp4");

    if( !capture.isOpened() ){
		return -1;
    }

	//capture.set(CV_CAP_PROP_FRAME_WIDTH,1280);
	//capture.set(CV_CAP_PROP_FRAME_HEIGHT,720);
	//capture.set(CV_CAP_PROP_FOURCC,CV_FOURCC('M','J','P','G'));
	//capture.set(CV_CAP_PROP_FPS, 30);
	cv::Mat image_camera;

	capture >> image_camera;
	std::cout<< capture.get(CV_CAP_PROP_FOURCC)								<< std::endl;
	
	std::cout<< "SET CODEC" << capture.set(CV_CAP_PROP_FOURCC,CV_FOURCC('M','J','P','G'))	<< std::endl;
	//std::cout<< "SET CODEC" << capture.set(CV_CAP_PROP_FOURCC,CV_FOURCC('M','P','4','2'))	<< std::endl;
	//std::cout<< "SET CODEC" << capture.set(CV_CAP_PROP_FOURCC,CV_FOURCC('D','I','V','3'))	<< std::endl;
	//std::cout<< "SET CODEC" << capture.set(CV_CAP_PROP_FOURCC,CV_FOURCC('D','I','V','X'))	<< std::endl;
	//std::cout<< "SET CODEC" << capture.set(CV_CAP_PROP_FOURCC,CV_FOURCC('U','2','6','3'))	<< std::endl;
	//std::cout<< "SET CODEC" << capture.set(CV_CAP_PROP_FOURCC,CV_FOURCC('I','2','6','3'))	<< std::endl;
	//std::cout<< "SET CODEC" << capture.set(CV_CAP_PROP_FOURCC,CV_FOURCC('F','L','V','1'))	<< std::endl;
	//std::cout<< "SET CODEC" << capture.set(CV_CAP_PROP_FOURCC,CV_FOURCC('X','2','6','4'))	<< std::endl;

	std::cout<< capture.set(CV_CAP_PROP_FPS, 30)							<< std::endl;

	std::cout<< "capture.set(CV_CAP_PROP_FRAME_WIDTH , 1920)"	<< 
		capture.set(CV_CAP_PROP_FRAME_WIDTH , 1920)				<< std::endl;
	std::cout<< "capture.set(CV_CAP_PROP_FRAME_HEIGHT, 1080)"	<<
		capture.set(CV_CAP_PROP_FRAME_HEIGHT, 1080)				<< std::endl;

	_sleep(100);

	
	

	//std::cout<< capture.set(CV_CAP_PROP_FPS, 30)							<< std::endl;
	//std::cout<< capture.get(CV_CAP_PROP_FOURCC)								<< std::endl;
	//

	printf("\n\n\n");
	std::cout	<<	"Cam Width - "	<<	capture.get(CV_CAP_PROP_FRAME_WIDTH)	<< std::endl;
	std::cout	<<	"Cam Height - "	<<	capture.get(CV_CAP_PROP_FRAME_HEIGHT)	<< std::endl;
	std::cout	<<	"Cam FPS - "	<<	capture.get(CV_CAP_PROP_FPS)			<< std::endl;
	std::cout	<<	"Cam Codec - "	<<	capture.get(CV_CAP_PROP_FOURCC)			<< std::endl;

	//int frames = capture.get(CV_CAP_PROP_FRAME_COUNT);
	//int cam_width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
	//int cam_height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);

	//
	//cv::VideoWriter record("recording_1080.avi", CV_FOURCC('D','I','V','X'), 30, cv::Size((int)video[0].get(CV_CAP_PROP_FRAME_WIDTH),(int)video[0].get(CV_CAP_PROP_FRAME_HEIGHT)), true);
	//cv::VideoWriter record("recording_1080.avi", CV_FOURCC('D','I','V','X'), 30, cv::Size((int)capture.get(CV_CAP_PROP_FRAME_WIDTH),(int)capture.get(CV_CAP_PROP_FRAME_HEIGHT)), true);

	//if(record_flag){
		////std::string::size_type pAt = source.find_last_of('.');                  // Find extension point
		//const std::string NAME = "Result.avi";//source.substr(0, pAt) + argv[2][0] + ".avi";   // Form the new name with container
		//int ex = static_cast<int>(video[0].get(CV_CAP_PROP_FOURCC));     // Get Codec Type- Int form

		//// Transform from int to char via Bitwise operators
		//char EXT[] = {(char)(ex & 0XFF) , (char)((ex & 0XFF00) >> 8),(char)((ex & 0XFF0000) >> 16),(char)((ex & 0XFF000000) >> 24), 0};

		//cv::Size S = cv::Size(	(int) video[0].get(CV_CAP_PROP_FRAME_WIDTH),    // Acquire input size
		//						(int) video[0].get(CV_CAP_PROP_FRAME_HEIGHT));

	

		//record.open(NAME, ex=-1, video[0].get(CV_CAP_PROP_FPS), S, true);

		////cv::VideoWriter record("Result.avi", CV_FOURCC('D','I','V','X'), 30, cv::Size(270,480), true);
		////cv::VideoWriter record("Result.avi", -1, 30, cv::Size(270,480), true);
	//}

	char c = 0;

//	cv::Mat image_camera;
	cv::namedWindow("win", CV_WINDOW_NORMAL);
    cv::setWindowProperty("win", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	while((c = cv::waitKey(11)) != 27){
		capture >> image_camera;

        if( image_camera.empty() )
            break;
		
		cv::imshow("win",image_camera);

		//record.write(image_camera);

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

	capture.release();

	return 0;
}