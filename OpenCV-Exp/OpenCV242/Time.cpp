#include <opencv2\opencv.hpp>

int main(){
	int64 now, then; 
	double elapsed_seconds,elapsed_milli_seconds, tickspersecond=cvGetTickFrequency() * 1.0e6,tickspermillisecond=cvGetTickFrequency() * 1.0e3;  

	cv::Point3f pt1(1,1,1);
	cv::Point3f pt2(2,2,2);

	cv::Vec3f v1 = pt1.operator cv::Vec<float, 3>();
	cv::Vec3f v2 = pt2.operator cv::Vec<float, 3>();

	cv::Point3f ptd = pt1-pt2;

	cv::Vec3f vn = cv::normalize(pt1-pt2).operator cv::Vec<float, 3>());

	double n = cv::norm(vn.operator cv::Matx<float, 3, 1>());
	double n2 = cv::norm(v1.operator cv::Matx<float, 3, 1>());

	while(1) { 
		then = cvGetTickCount(); 
		Sleep(1000);
		now = cvGetTickCount();
		int64 diff = now-then;
		elapsed_seconds = ((double)(diff) / (double)tickspersecond); 
		elapsed_milli_seconds = ((double)(diff) / (double)tickspermillisecond); 

		printf("%.6f seconds",elapsed_seconds);
		printf("\t%.2f miliseconds\n",elapsed_milli_seconds);
	}


	return 0;
}	