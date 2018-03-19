#include "ChromaKey.h"

#include <opencv2\opencv.hpp>

int _red = 8, _green = 150, _blue = 150;
int _threashold = 68;

double  _h_min = 47, _h_max = 68, _s_min, _s_max,_v_min, _v_max;
int _h_min_slider = 47, _h_max_slider=68,_s_min_slider, _s_max_slider,_v_min_slider, _v_max_slider;
int _h_min_slider_max = 255, _h_max_slider_max = 255,_s_min_slider_max = 255, _s_max_slider_max = 255,_v_min_slider_max= 255, _v_max_slider_max= 255;

void on_trackbar_h_min( int, void* )
{
	//_h_min = (double) _h_min_slider/_h_min_slider_max ;
	_h_min = (double) _h_min_slider;
}

void on_trackbar_h_max( int, void* )
{
	//_h_max = (double) _h_max_slider/_h_max_slider_max ;
	_h_max = (double) _h_max_slider;
}

void on_trackbar_s_min( int, void* )
{
	//_s_min = (double) _s_min_slider/_s_min_slider_max ;
	_s_min = (double) _s_min_slider;
}

void on_trackbar_s_max( int, void* )
{
	//_s_max = (double) _s_max_slider/_s_max_slider_max ;
	_s_max = (double) _s_max_slider;
}

void on_trackbar_v_min( int, void* )
{
	//_v_min = (double) _v_min_slider/_v_min_slider_max ;
	_v_min = (double) _v_min_slider;
}

void on_trackbar_v_max( int, void* )
{
	//_v_max = (double) _v_max_slider/_v_max_slider_max ;
	_v_max = (double) _v_max_slider;
}

const int n_videos = 1;

int main_chroma_key(int argc, char* argv[]){
	cv::setUseOptimized(true);

	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;


	cv::VideoCapture capture;

	//capture.open(-1);
	capture.open("vid.mp4");

    if( !capture.isOpened() ){
		return -1;
    }

	//capture.set(CV_CAP_PROP_FRAME_WIDTH,1920);
	//capture.set(CV_CAP_PROP_FRAME_HEIGHT,1080);

	char c = 0;

	cv::Mat image_camera;
	cv::Mat image_video[n_videos], image_video_hsv[n_videos];
	cv::Mat video_mask[n_videos];

	cv::namedWindow("Controls");
	cv::createTrackbar("H Min","Controls",&_h_min_slider,_h_min_slider_max,on_trackbar_h_min);
	cv::createTrackbar("H Max","Controls",&_h_max_slider,_h_max_slider_max,on_trackbar_h_max);
	cv::createTrackbar("S Min","Controls",&_s_min_slider,_s_min_slider_max,on_trackbar_s_min);
	cv::createTrackbar("S Max","Controls",&_s_max_slider,_s_max_slider_max,on_trackbar_s_max);
	cv::createTrackbar("V Min","Controls",&_v_min_slider,_v_min_slider_max,on_trackbar_v_min);
	cv::createTrackbar("V Max","Controls",&_v_max_slider,_v_max_slider_max,on_trackbar_v_max);
	cv::moveWindow("Controls",1400,50);
	
	cv::namedWindow("win", CV_WINDOW_NORMAL);
    cv::setWindowProperty("win", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	while((c = cv::waitKey(11)) != 27){
		capture >> image_camera;


        if( image_camera.empty() )
            break;

		for(int i = 0 ; i < n_videos ; ++i){
			//cv::cvtColor( image_video[i], image_video_hsv[i], CV_BGR2HSV );
			image_video[i].copyTo(image_video_hsv[i]);

			video_mask[i] = cv::Mat::ones(image_video_hsv[i].rows, image_video_hsv[i].cols, CV_8UC1);

			unsigned char* ptr_clr = image_video_hsv[i].data;

		//	unsigned char* ptr_clr = image_video_hsv[i].data;
			unsigned char* ptr_mask = video_mask[i].data;

			for(int y = 0 ; y < image_video_hsv[i].rows ; y++) { 
				for(int x = 0; x < image_video_hsv[i].cols ; x++) { 
					int mpos = y * image_video_hsv[i].cols + x;
					//int hpos = y * image_video_hsv[i].cols * 3 + x * 3 + 0;
					int spos = y * image_video_hsv[i].cols * 3 + x * 3 + 1;
					//int vpos = y * image_video_hsv[i].cols * 3 + x * 3 + 2;

					if(ptr_clr[spos] > 240){
						ptr_mask[mpos] = 0;
					}
				}
			}



			//for(int y = 0 ; y < image_video_hsv[i].rows ; y++) { 
			//	for(int x = 0; x < image_video_hsv[i].cols ; x++) { 
			//		int mpos = y * image_video_hsv[i].cols + x;
			//		int hpos = y * image_video_hsv[i].cols * 3 + x * 3 + 0;
			//		int spos = y * image_video_hsv[i].cols * 3 + x * 3 + 1;
			//		int vpos = y * image_video_hsv[i].cols * 3 + x * 3 + 2;

			//		if(ptr_clr[hpos] > _h_min && ptr_clr[hpos] < _h_max){
			//			ptr_mask[mpos] = 0;
			//		} else
			//		if(ptr_clr[spos] > _s_min && ptr_clr[spos] < _s_max){
			//			ptr_mask[mpos] = 0;
			//		} else
			//		if(ptr_clr[vpos] > _v_min && ptr_clr[vpos] < _v_max){
			//			ptr_mask[mpos] = 0;
			//		}
			//	}
			//}

			//image_video_hsv[i].copyTo(image_camera(cv::Rect(init_points[i].x,init_points[i].y,image_video[i].cols, image_video[i].rows)),video_mask[i]);
		}

		cv::imshow("Controls",video_mask[0]);
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

	//record.~VideoWriter();

	capture.release();

	return 0;
}