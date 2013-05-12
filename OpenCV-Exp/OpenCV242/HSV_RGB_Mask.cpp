#include <opencv2\opencv.hpp>

double  _h_min, _h_max, _s_min, _s_max,_v_min, _v_max;
int _h_min_slider, _h_max_slider,_s_min_slider, _s_max_slider,_v_min_slider, _v_max_slider;
int _h_min_slider_max = 255, _h_max_slider_max = 255,_s_min_slider_max = 255, _s_max_slider_max = 255,_v_min_slider_max= 255, _v_max_slider_max= 255;

void on_trackbar_h_min( int, void* )
{
	_h_min = (double) _h_min_slider;
}

void on_trackbar_h_max( int, void* )
{
	_h_max = (double) _h_max_slider;
}

void on_trackbar_s_min( int, void* )
{
	_s_min = (double) _s_min_slider;
}

void on_trackbar_s_max( int, void* )
{
	_s_max = (double) _s_max_slider;
}

void on_trackbar_v_min( int, void* )
{
	_v_min = (double) _v_min_slider;
}

void on_trackbar_v_max( int, void* )
{
	_v_max = (double) _v_max_slider;
}

int main(){
    // capture frame from video file 
    CvCapture* capture_orig = cvCreateFileCapture( "vid.mp4");
	CvCapture* capture = capture_orig;

    // Create IplImage to point to each frame 
	IplImage* frame;
	IplImage* frame_processed;
	IplImage* frame_back;
    
	frame = cvQueryFrame(capture);
	frame_processed = cvCloneImage(frame);
	frame_back = cvCloneImage(frame);

	CvScalar _black;
	_black.val[0] = 0;
	_black.val[1] = 0;
	_black.val[2] = 0;
	
	IplImage* hsv = cvCreateImage( cvGetSize(frame), IPL_DEPTH_8U, 3 );
	cvCvtColor( frame, hsv, CV_BGR2HSV );

	double fps = cvGetCaptureProperty(capture,CV_CAP_PROP_FPS);
	CvSize size = cvSize(cvGetCaptureProperty(capture,CV_CAP_PROP_FRAME_WIDTH ),cvGetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT ));
	
	cvNamedWindow("TrackBar Window", CV_WINDOW_AUTOSIZE);
	IplImage* trackbar = cvCreateImage(cvSize(640,20), IPL_DEPTH_8U, 3);

	for(int i = 0 ; i < 640 ; i++){
		for(int j = 0 ; j < 20; j++){
			cvSet2D(trackbar,j,i,_black);
		}
	}

	CvVideoWriter* writer = cvCreateVideoWriter("Video from Images.avi",CV_FOURCC('D','I','V','X'),fps,size);
	cv::createTrackbar("H Min","TrackBar Window",&_h_min_slider,_h_min_slider_max,on_trackbar_h_min);
	cv::createTrackbar("H Max","TrackBar Window",&_h_max_slider,_h_max_slider_max,on_trackbar_h_max);
	cv::createTrackbar("S Min","TrackBar Window",&_s_min_slider,_s_min_slider_max,on_trackbar_s_min);
	cv::createTrackbar("S Max","TrackBar Window",&_s_max_slider,_s_max_slider_max,on_trackbar_s_max);
	cv::createTrackbar("V Min","TrackBar Window",&_v_min_slider,_v_min_slider_max,on_trackbar_v_min);
	cv::createTrackbar("V Max","TrackBar Window",&_v_max_slider,_v_max_slider_max,on_trackbar_v_max);
	cvShowImage("TrackBar Window", trackbar);

	cvNamedWindow("Example2", CV_WINDOW_AUTOSIZE);

	char c;
	int ac = 0;
    while(1) {
		c = cvWaitKey(33);
		
        if(c==27) break;

		// grab frame image, and retrieve 
		frame_back = cvQueryFrame(capture);
		frame = cvCloneImage(frame_back);
		frame_processed = cvCloneImage(frame_back);
		
		if(!frame)  break;

		cvCvtColor( frame_back, hsv, CV_BGR2HSV );

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
			}
		}

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
	cvDestroyWindow("TrackBar Window");

	return 0;
}