#include <opencv2\opencv.hpp>
#include <time.h>
#include <Windows.h>

int main(){
	cv::Mat1b mask;
	cv::Mat3b depth_aux;
	cv::Mat depth;


	mask = cv::imread("mask.png",0);
	depth_aux = cv::imread("depth.png");

	clock_t Start = clock();

	Sleep(1000);

	clock_t Start2 = clock();
	
	SYSTEMTIME stime;
	SYSTEMTIME stime2;
	FILETIME ftime;
	FILETIME ftime2;
	FILETIME ftime3;
	GetSystemTimeAsFileTime(&ftime);
	Sleep(2000);
	GetSystemTimeAsFileTime(&ftime2);

	long asdasd = ftime2.dwLowDateTime- ftime.dwLowDateTime; //microsec
	ftime3.dwLowDateTime = ftime.dwLowDateTime + asdasd;
	ftime3.dwHighDateTime = ftime.dwHighDateTime;

	FileTimeToSystemTime(&ftime3,&stime);
	FileTimeToSystemTime(&ftime,&stime2);

	//SystemTimeToFileTime

	time_t asd = time(0);//(double)clock()/CLOCKS_PER_SEC;//cv::getTickCount() / cv::getTickFrequency();


	stime = stime2;

	struct tm * now = localtime( & asd );
	now->tm_hour;
	now->tm_min;
	now->tm_year;
	double minutes = asd / 60.0;

	double hours = minutes / 60.0;

	double days = hours / 24.0;

	double years = days / 365.25;

	SYSTEMTIME st;
	SYSTEMTIME st2;
	SYSTEMTIME st3;
	st2.wMilliseconds = 1000;
	GetLocalTime(&st);

	st3 = st;

	st3.wMilliseconds += st2.wMilliseconds;




	cv::imshow("mask",mask);
	cv::imshow("depth",depth_aux);

	cv::waitKey();

	depth_aux.convertTo(depth,CV_8U);

	cv::imshow("depth",depth);
	cv::waitKey();

	return 0;
}

//#include <opencv2\opencv.hpp>
//
//int main(){
//	cv::Mat3b user_labels;
//
//	user_labels = cv::imread("gg.png");
//
//	cv::vector<cv::vector<cv::Point> > contours;
//    cv::vector<cv::Vec4i> hierarchy;
//    cv::Mat1b threshold_output;
//    cv::Mat1b src_gray;
//    cvtColor( user_labels, src_gray, CV_BGR2GRAY );
//    /// Detect edges using Threshold
//    cv::threshold( src_gray, threshold_output, 100, 255, cv::THRESH_BINARY );
//    /// Find contours
//    cv::findContours( threshold_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
//
//	cv::imshow("1",user_labels);
//	cv::waitKey();
//
//    cv::vector<cv::Rect> boundRect( contours.size() );
//    for( unsigned int i = 0; i < contours.size(); i++ )
//    {// approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
//        if(contours[i].size() > 50){
//            boundRect[i] = boundingRect( cv::Mat(contours[i]) );
//            //cv::circle(user_labels, cv::Point(boundRect[i].x,boundRect[i].y),5,cv::Scalar(0,255,0),3);
//            //cv::circle(user_labels, cv::Point(boundRect[i].x+boundRect[i].width,boundRect[i].y),5,cv::Scalar(0,0,255),3);
//            //cv::rectangle( user_labels, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(0,255,0), 2, 8, 0 );
//
//            bool found = false;
//			unsigned char* data = (unsigned char*)src_gray.data;
//
//			int asdasd = data[src_gray.step * 300 + 100];
//
//            for(int a = 0 ; a < 10 && !found ; a++){
//                for(int k = 0 ; k < boundRect[i].width && !found ; k++){
//                    int aux_x = boundRect[i].x+k;
//                    int aux_y = boundRect[i].y+a;
//
//					int asdasd = data[src_gray.step * aux_y + aux_x];
//                    //cv::circle(src_gray, cv::Point(aux_x,aux_y),1,cv::Scalar(200,200,200));
//
//                    //cv::imshow("31",src_gray);
//                    char value =src_gray.ptr<unsigned char>(aux_y)[aux_x];
//
//					cv::imshow("1",src_gray);
//					cv::waitKey(5);
//
//					printf("v: %d %d\n",value,asdasd);
//
//                    if(src_gray.ptr<unsigned char>(aux_y)[aux_x]){
//                        cv::circle(user_labels, cv::Point(aux_x,aux_y),5,cv::Scalar(0,255,0),3);
//
//                        found = true;
//                    }
//                }
//            }
//        }
//    }
//
//	cv::imshow("1",user_labels);
//	cv::waitKey();
//
//
//	return 0;
//}



//cv::Mat image;
//cv::Mat blur;
//cv::Mat erode;
//cv::Mat dilate;
//cv::Mat bin;
//
//cv::Mat element_3(3,3,CV_8U,cv::Scalar(1));
//cv::Mat element_5(5,5,CV_8U,cv::Scalar(1));
//cv::Mat element_7(7,7,CV_8U,cv::Scalar(1));
//cv::Mat element_9(9,9,CV_8U,cv::Scalar(1));
//cv::Mat element_11(11,11,CV_8U,cv::Scalar(1));
//
//cv::Mat element_3_15(3,15,CV_8U,cv::Scalar(1));
//cv::Mat element_7_3(7,3,CV_8U,cv::Scalar(1));
//
//int main(){
//
//	image = cv::imread("map.bmp");
//
//	cv::blur(image,blur,cv::Size(3,3));
//
//	cv::dilate(image,dilate,element_3_15);
//	cv::dilate(dilate,dilate,element_7_3);
//
//	cv::erode(dilate,erode,element_7);
//
//	cv::dilate(erode,erode,element_3);
//
//	cv::threshold(erode,bin,1,255,CV_THRESH_BINARY);
//
//	cv::imshow("orig",image);
//	cv::imshow("blur",blur);
//	cv::imshow("dilate",dilate);
//	cv::imshow("erode",erode);
//	cv::imshow("bin",bin);
//
//	cv::waitKey();
//
//	return 0;
//}
//
//double degree_to_rad(double deg){
//	return deg * 0.0174532925;
//}
//
//int main(){
//	//cv::Mat3b img(2560,1920,cv::Vec3f(255,255,255));
//
//	//cv::Point center(960,1280);
//	//cv::Point old;
//
//	//cv::Point end;
//	//int length = 960;
//	//for(int i = 0 ; i < 360 ; i+=10){
//	//	end.x = (sin(degree_to_rad(i)) * length) + center.x;
//	//	end.y = (cos(degree_to_rad(i)) * length) + center.y;
//	//	
//	//	cv::line(img,center,end,cv::Scalar(255,0,0),3);
//	//	
//	//	if(i){
//	//		
//	//		cv::line(img,old,end,cv::Scalar(254,254,254),3);
//	//		old = end;
//	//	}
//	//	else{
//	//		old = end;
//	//	}
//
//	//}
//
//	//end.x = (sin(degree_to_rad(0)) * length) + center.x;
//	//end.y = (cos(degree_to_rad(0)) * length) + center.y;
//	//cv::line(img,old,end,cv::Scalar(254,254,254),3);
//
//	//for(int i = 0 ; i < 360 ; i+=10){
//	//	end.x = (sin(degree_to_rad(i)) * length) + center.x;
//	//	end.y = (cos(degree_to_rad(i)) * length) + center.y;
//
//	//	cv::line(img,end,end,cv::Scalar(253,253,253),7);
//	//}
//
//	//cv::imshow("asd",img);
//	//cv::imwrite("img.png",img);
//	//cv::waitKey();
//
//	//cv::Mat img = cv::imread("dn2.jpg");
//
//	//for(float i = 0 ; i < img.size().width ; i+= (((double)img.size().width))/18.0){
//	//	cv::line(img,cv::Point(i,0),cv::Point(i,img.size().height),cv::Scalar(255,255,255));
//
//	//}
//
//	//cv::imshow("sa",img);
//	//cv::imwrite("dn.png",img);
//	//cv::waitKey();
//
//	cv::Mat3b img = cv::imread("img_q.PNG");
//	cv::Mat img_f;
//	cv::Mat1b img_b(img.size());
//	cv::Mat1f coiso;
//
//	cv::Mat3b out(img.size());
//
//	cv::cvtColor( img, img_f, CV_BGR2GRAY );
//	cv::threshold(img_f,img_b,100,255,cv::THRESH_BINARY);
//
//	img_b.convertTo(coiso,CV_32F);
//
//	cv::vector<cv::vector<cv::Point> > contours;
//          cv::vector<cv::Vec4i> hierarchy;
//
//		 // img_b.convertTo(coiso,CV_32F);
//
//        cv::Mat threshold_output;
//
//		
//		cv::findContours( img_b, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
//
//		for( int i = 0; i < contours.size(); i++ ){
//			cv::drawContours(out,contours,i,cv::Scalar(255,0,0),33);
//		}
//
//
//	cv::imshow("sa",out);
//	//cv::imwrite("img_q_2.png",img);
//	cv::waitKey();
//
//	return 0;
//}