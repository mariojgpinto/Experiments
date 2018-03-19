#include <opencv2\opencv.hpp>

int main_video_mask_alpha_join(int argc, char* argv[]) {

	int _black_min = 20; 
	int _black_max = 50;
	cv::namedWindow("Color");
	cv::createTrackbar("Black", "Color", &_black_min, _black_max);


	//CvCapture* capture_mask = cvCreateFileCapture("Split/01BW.mp4");
	//CvCapture* capture_color = cvCreateFileCapture("Split/01color.mp4");
	//CvCapture* capture_mask = cvCreateFileCapture("Split/02BW.mp4");
	//CvCapture* capture_color = cvCreateFileCapture("Split/02color.mp4");
	//CvCapture* capture_mask = cvCreateFileCapture("Split/03BW.mp4");
	//CvCapture* capture_color = cvCreateFileCapture("Split/03color.mp4");
	CvCapture* capture_mask = cvCreateFileCapture("Split/04BW.mp4");
	CvCapture* capture_color = cvCreateFileCapture("Split/04color.mp4");

	// Create IplImage to point to each frame 
	IplImage* frame_color;
	IplImage* frame_mask;
	IplImage* frame_processed;

	// Loop until frame ended or ESC is pressed 
	frame_color = cvQueryFrame(capture_color);
	frame_mask = cvQueryFrame(capture_mask);
	frame_processed = cvCloneImage(frame_color);

	printf("Color:%dx%d\n", frame_color->width, frame_color->height);
	printf("Mask:%dx%d\n", frame_mask->width, frame_mask->height);

	double fps = cvGetCaptureProperty(capture_color, CV_CAP_PROP_FPS);
	CvSize size = cvSize(cvGetCaptureProperty(capture_color, CV_CAP_PROP_FRAME_WIDTH), cvGetCaptureProperty(capture_color, CV_CAP_PROP_FRAME_HEIGHT));
	CvVideoWriter* writer = cvCreateVideoWriter("Split/result04.avi", CV_FOURCC('D', 'I', 'V', 'X'), fps, size);

	CvScalar _green;
	_green.val[0] = 0;
	_green.val[1] = 255;
	_green.val[2] = 0;

	char c;
	int ac = 0;
	while (1) {
		c = cvWaitKey(33);
		//processKey(c);
		if (c == 27) break;

		//if (c == 'n') {
		//	frame_color = cvQueryFrame(capture_color); 
		//	frame_mask = cvQueryFrame(capture_mask);
		//	frame_processed = cvCloneImage(frame_color);
		//}
		//else {

			frame_color = cvQueryFrame(capture_color);
			frame_mask = cvQueryFrame(capture_mask);

			if (frame_color == NULL) {
				printf("NULL");
			}
			// exit loop if fram is null / movie end 
			if (!frame_color || !frame_mask)  break;

			frame_processed = cvCloneImage(frame_color);
		//}


		

		for( int i = 0; i < frame_mask->height; i++ ) {
			for( int j = 0; j < frame_mask->width-1; j++ ) {
				CvScalar _aux; 
				_aux=cvGet2D(frame_mask,i,j);

				if(_aux.val[0] <= _black_min && _aux.val[1] <= _black_min && _aux.val[2] <= _black_min){
					cvSet2D(frame_processed, i, j, _green);
				}
			}
		}

		//for (int i = 0; i < frame_color->height; i++) {
		//	for (int j = frame_mask->width-4; j < frame_mask->width; j++) {
		//		cvSet2D(frame_processed, i, j, _green);
		//	}
		//}

		//for (int i = 0; i < 2; i++) {
		//	for (int j = 0; j < frame_mask->width; j++) {
		//		cvSet2D(frame_processed, i, j, _green);
		//	}
		//}


		cvShowImage("Color", frame_color);
		cvShowImage("Mask", frame_mask);
		cvShowImage("Processed", frame_processed);

		cvWriteFrame(writer, frame_processed);

		cvReleaseImage(&frame_processed);
		//cvReleaseImage(&frame_mask);
	}

	cvReleaseVideoWriter(&writer);

	// destroy pointer to video 
	cvReleaseCapture(&capture_color);
	cvReleaseCapture(&capture_mask);
	// delete window 
	//cvDestroyWindow("Example2");


	return 0;
}