#include <opencv2\opencv.hpp>

int _red = 8, _green = 150, _blue = 150;
int _threashold = 68;

double  _h_min, _h_max, _s_min, _s_max,_v_min, _v_max;
int _h_min_slider, _h_max_slider,_s_min_slider, _s_max_slider,_v_min_slider, _v_max_slider;
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

void processKey(char c){
	switch(c){
		case '1':
			_red++;
			std::cout << "red: " << _red << "\tgreen:" << _green << "\tblue:" << _blue << "\n";
			break;
		case '2':
			_red--;
			std::cout << "red: " << _red << "\tgreen:" << _green << "\tblue:" << _blue << "\n";
			break;
		case '3':
			_green++;
			std::cout << "red: " << _red << "\tgreen:" << _green << "\tblue:" << _blue << "\n";
			break;
		case '4':
			_green--;
			std::cout << "red: " << _red << "\tgreen:" << _green << "\tblue:" << _blue << "\n";
			break;
		case '5':
			_blue++;
			std::cout << "red: " << _red << "\tgreen:" << _green << "\tblue:" << _blue << "\n";
			break;
		case '6':
			_blue--;
			std::cout << "red: " << _red << "\tgreen:" << _green << "\tblue:" << _blue << "\n";
			break;
		case '7':
			_threashold++;
			std::cout << "threashold: " << _threashold << "\n";
			break;
		case '8':
			_threashold--;
			std::cout << "threashold: " << _threashold << "\n";
			break;
		default: break;
	}
}

int main(){
	// Create a window 
    cvNamedWindow("Example2", CV_WINDOW_AUTOSIZE);
    // capture frame from video file 
    CvCapture* capture_orig = cvCreateFileCapture( "vid.mp4");
	CvCapture* capture = capture_orig;
    // Create IplImage to point to each frame 
    IplImage* frame;
	IplImage* frame_processed;
	IplImage* frame_back;
    // Loop until frame ended or ESC is pressed 
	frame = cvQueryFrame(capture);
	frame_processed = cvCloneImage(frame);
	frame_back = cvCloneImage(frame);


	CvScalar _black;
	_black.val[0] = 0;
	_black.val[1] = 0;
	_black.val[2] = 0;

	IplImage* hsv = cvCreateImage( cvGetSize(frame), IPL_DEPTH_8U, 3 );
	cvCvtColor( frame, hsv, CV_BGR2HSV );
	//CvScalar s; 
	//s=cvGet2D(hsv,300,400);

	double fps = cvGetCaptureProperty(capture,CV_CAP_PROP_FPS);
	CvSize size = cvSize(cvGetCaptureProperty(capture,CV_CAP_PROP_FRAME_WIDTH ),cvGetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT ));
			
	CvVideoWriter* writer = cvCreateVideoWriter("Video from Images.avi",CV_FOURCC('D','I','V','X'),fps,size);
	cv::createTrackbar("H Min","Example2",&_h_min_slider,_h_min_slider_max,on_trackbar_h_min);
	cv::createTrackbar("H Max","Example2",&_h_max_slider,_h_max_slider_max,on_trackbar_h_max);
	cv::createTrackbar("S Min","Example2",&_s_min_slider,_s_min_slider_max,on_trackbar_s_min);
	cv::createTrackbar("S Max","Example2",&_s_max_slider,_s_max_slider_max,on_trackbar_s_max);
	cv::createTrackbar("V Min","Example2",&_v_min_slider,_v_min_slider_max,on_trackbar_v_min);
	cv::createTrackbar("V Max","Example2",&_v_max_slider,_v_max_slider_max,on_trackbar_v_max);

	char c;
	int ac = 0;
    while(1) {
		c = cvWaitKey(33);
		processKey(c);
        if(c==27) break;
        // grab frame image, and retrieve 
		//if (ac == 1){
		//frame = cvQueryFrame(capture);
		if(c == 'n'){
//			cvReleaseImage(&frame_back);
			frame_back = cvQueryFrame(capture);
			frame = cvCloneImage(frame_back);
			frame_processed = cvCloneImage(frame_back);
		}
		else{
			
			frame = cvCloneImage(frame_back);
			frame_processed = cvCloneImage(frame_back);
		}
		//ac = 0;
		//}
		//ac++;
        // exit loop if fram is null / movie end 
        if(!frame)  break;
		//{
		//	capture = cvCreateFileCapture( "F15.mp4");
		//	frame = cvQueryFrame(capture);
		//}

		cvCvtColor( frame_back, hsv, CV_BGR2HSV );

		//for( int i = 0; i < frame->height; i++ ) { 
		//	for( int j = 0; j < frame->width; j++ ) { 
		//		CvScalar _aux; 
		//		_aux=cvGet2D(frame,i,j);

		//		/*if( abs(_aux.val[0] - s.val[0]) < _threashold && 
		//			abs(_aux.val[1] - s.val[1]) < _threashold && 
		//			abs(_aux.val[2] - s.val[2]) < _threashold){*/
		//		if(sqrt(pow(_aux.val[0] - s.val[0],2) + 
		//				pow(_aux.val[1] - s.val[1],2) +
		//				pow(_aux.val[2] - s.val[2],2)) < _threashold){
		//				cvSet2D(frame,i,j,_black);
		//		}

		//	}
		//}
		//for( int y=0; y<frame->height; y++ ) { 
		//	uchar* ptr = (uchar*) ( frame->imageData + y * frame->widthStep ); 
		//	for( int x=0; x<frame->width; x++ ) { 
		//		if((ptr[3*x+1] > _green) && (ptr[3*x+0] < _red) && (ptr[3*x+0] < _blue) ){ //Set red to max (BGR format)
		//			ptr[3*x+0] = 0;
		//			ptr[3*x+1] = 0;
		//			ptr[3*x+2] = 0;
		//		}
		//		//if(sqrt(pow(ptr[3*x+0] - s.val[0],2) + 
		//		//		pow(ptr[3*x+1] - s.val[1],2) +
		//		//		pow(ptr[3*x+2] - s.val[2],2)) < _threashold){
		//		//	ptr[3*x+0] = 0;
		//		//	ptr[3*x+1] = 0;
		//		//	ptr[3*x+2] = 0;
		//		//}

		//	}
		//}

		for( int y=0; y<hsv->height; y++ ) { 
			uchar* ptr_hsv = (uchar*) ( hsv->imageData + y * hsv->widthStep ); 
			uchar* ptr_rgb = (uchar*) ( frame->imageData + y * frame->widthStep ); 
			uchar* ptr_rgb_alt = (uchar*) ( frame_processed->imageData + y * frame_processed->widthStep ); 
			for( int x=0; x<hsv->width; x++ ) { 
				if(ptr_hsv[3*x+0] > _h_min && ptr_hsv[3*x+0] < _h_max){
					ptr_rgb_alt[3*x+0] = 0;
					ptr_rgb_alt[3*x+1] = 0;
					ptr_rgb_alt[3*x+2] = 0;
				}else
				if(ptr_hsv[3*x+1] > _s_min && ptr_hsv[3*x+1] < _s_max){
					ptr_rgb_alt[3*x+0] = 0;
					ptr_rgb_alt[3*x+1] = 0;
					ptr_rgb_alt[3*x+2] = 0;
				}else
				if(ptr_hsv[3*x+2] > _v_min && ptr_hsv[3*x+2] < _v_max){
					ptr_rgb_alt[3*x+0] = 0;
					ptr_rgb_alt[3*x+1] = 0;
					ptr_rgb_alt[3*x+2] = 0;
				}else{
					ptr_rgb_alt[3*x+0] = ptr_rgb[3*x+0];
					ptr_rgb_alt[3*x+1] = ptr_rgb[3*x+1];
					ptr_rgb_alt[3*x+2] = ptr_rgb[3*x+2];
				}
				//if((ptr[3*x+1] > _green) && (ptr[3*x+0] < _red) && (ptr[3*x+0] < _blue) ){ //Set red to max (BGR format)
				//	ptr[3*x+0] = 0;
				//	ptr[3*x+1] = 0;
				//	ptr[3*x+2] = 0;
				//}
				//if(sqrt(pow(ptr[3*x+0] - s.val[0],2) + 
				//		pow(ptr[3*x+1] - s.val[1],2) +
				//		pow(ptr[3*x+2] - s.val[2],2)) < _threashold){
				//	ptr[3*x+0] = 0;
				//	ptr[3*x+1] = 0;
				//	ptr[3*x+2] = 0;
				//}
				//if(	abs(ptr[3*x+0] - s.val[0]) < _h_min &&
				//	abs(ptr[3*x+1] - s.val[1]) < _green &&
				//	abs(ptr[3*x+2] - s.val[2]) < _blue ){
				//		ptr_rgb[3*x+0] = 0;
				//		ptr_rgb[3*x+1] = 0;
				//		ptr_rgb[3*x+2] = 0;
				//}
			}
		}

		//cvCvtColor( hsv, frame , CV_HSV2BGR );//
        // display frame into window 
        cvShowImage("Example2", frame_processed);
		cvWriteFrame( writer, frame);

        // if ESC is pressed then exit loop 
        
		cvReleaseImage(&frame);
		cvReleaseImage(&frame_processed);
    }

	cvReleaseVideoWriter( &writer );

    // destroy pointer to video 
    cvReleaseCapture(&capture);
    // delete window 
    cvDestroyWindow("Example2");


	return 0;
}








/*
#include <opencv2\opencv.hpp>

int _red = 100, _green = 100, _blue = 100;

void processKey(char c){
	switch(c){
		case '1':
			_red++;
			std::cout << "red: " << _red << "\tgreen:" << _green << "\tblue:" << _blue << "\n";
			break;
		case '2':
			_red--;
			std::cout << "red: " << _red << "\tgreen:" << _green << "\tblue:" << _blue << "\n";
			break;
		case '3':
			_green++;
			std::cout << "red: " << _red << "\tgreen:" << _green << "\tblue:" << _blue << "\n";
			break;
		case '4':
			_green--;
			std::cout << "red: " << _red << "\tgreen:" << _green << "\tblue:" << _blue << "\n";
			break;
		case '5':
			_blue++;
			std::cout << "red: " << _red << "\tgreen:" << _green << "\tblue:" << _blue << "\n";
			break;
		case '6':
			_blue--;
			std::cout << "red: " << _red << "\tgreen:" << _green << "\tblue:" << _blue << "\n";
			break;
		default: break;
	}
}

int main(){
	// Create a window 
    cvNamedWindow("Example2", CV_WINDOW_AUTOSIZE);
    // capture frame from video file
    CvCapture* capture = cvCreateFileCapture( "vid.mp4");
    // Create IplImage to point to each frame
    IplImage* frame;
    // Loop until frame ended or ESC is pressed
	frame = cvQueryFrame(capture);



    while(1) {
        // grab frame image, and retrieve
        frame = cvQueryFrame(capture);
        // exit loop if fram is null / movie end
        if(!frame) break;

		for( int y=0; y<frame->height; y++ ) { 
			uchar* ptr = (uchar*) ( frame->imageData + y * frame->widthStep ); 
			for( int x=0; x<frame->width; x++ ) { 
				if((ptr[3*x+1] > _green) && (ptr[3*x+0] < _red) && (ptr[3*x+0] < _blue) ){ //Set red to max (BGR format)
					ptr[3*x+0] = 0;
					ptr[3*x+1] = 0;
					ptr[3*x+2] = 0;
				}
			}
		}

        // display frame into window
        cvShowImage("Example2", frame);
        // if ESC is pressed then exit loop
        char c = cvWaitKey(33);
		processKey(c);
        if(c==27) break;
    }
    // destroy pointer to video
    cvReleaseCapture(&capture);
    // delete window
    cvDestroyWindow("Example2");


	return 0;
}
*/