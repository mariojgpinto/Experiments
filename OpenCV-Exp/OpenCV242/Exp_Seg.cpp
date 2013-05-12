#include <opencv2\opencv.hpp>
#include <time.h>

#define THRESHOLD_RANGE 2

char c = 0;

int min = 150 ;
int max = 200;

cv::Mat img0;
cv::Mat1b imgGray;

cv::Mat1b imgGray_1;
cv::Mat1b imgGray_2;
cv::Mat1b imgGray_3;
cv::Mat1b imgGray_4;
cv::Mat1b imgGray_diff;
cv::Mat3b imgGray_sum;

cv::Mat1b object;
cv::Mat1b object_diff;

cv::Mat3b c_1;
cv::Mat3b c_2;
cv::Mat3b c_3;

bool turn = true;
int round = 255;

bool flag = false;
cv::Point click;

int fill_flag = 0;

#define PRINT_THRESH(min,max) printf("min:%d max:%d\n",(min),(max));

void process_key(char c){
	switch(c){
	case 27 : exit(0);
		case '1':
			min++;
			PRINT_THRESH(min,max)
			break;
		case '2':
			min--;
			PRINT_THRESH(min,max)
			break;
		case '3':
			max++;
			PRINT_THRESH(min,max)
			break;
		case '4':
			max--;
			PRINT_THRESH(min,max)
			break;
		case '.':
			min++;
			max++;
			PRINT_THRESH(min,max)
			break;
		case ',':
			min--;
			max--;
			PRINT_THRESH(min,max)
			break;
		default:break;
	}
}

/*
void fill_space(int x, int y){
	int xx = x,yy = y;

	if(x > 0 && y > 0 && x < img0.size().width && y < img0.size().height){
		if(object.at<uchar>(x,y) != round && imgGray_diff.at<uchar>(x,y)){
			object.at<uchar>(x,y) = round;
			
			fill_space(x+1, y  );
			fill_space(x+1, y+1);
			fill_space(x  , y+1);
			fill_space(x-1, y  );
			fill_space(x-1, y-1);
			fill_space(x  , y-1);
			fill_space(x+1, y-1);
			fill_space(x-1, y+1);

			fill_space(x+2, y  );
			fill_space(x+2, y+2);
			fill_space(x  , y+2);
			fill_space(x-2, y  );
			fill_space(x-2, y-2);
			fill_space(x  , y-2);
			fill_space(x+2, y-2);
			fill_space(x-2, y+2);

		}
	}
}
*/

void fill_space(int x, int y, int my_min, int my_max){
	//Check if in range
	//if(x > 0 && y > 0 && x < img0.size().width && y < img0.size().height){
		//check if already in mask
		//if(object.at<uchar>(y,x))
		//	return;

		//check if inside threshold
		uchar value = img0.at<uchar>(y,x);

		if(value >= my_min && value <= my_max){
			object.at<uchar>(y,x) = 255;

			//Check neighborhood
			int _min = value - THRESHOLD_RANGE;
			int _max = value + THRESHOLD_RANGE;

			//fill_space(x+1, y  ,_min,_max);
			//fill_space(x+1, y+1,_min,_max);
			//fill_space(x  , y+1,_min,_max);
			//fill_space(x-1, y  ,_min,_max);
			//fill_space(x-1, y-1,_min,_max);
			//fill_space(x  , y-1,_min,_max);
			//fill_space(x+1, y-1,_min,_max);
			//fill_space(x-1, y+1,_min,_max);

			if(!object.at<uchar>(y  ,x+1)) fill_space(x+1, y  ,_min,_max);
			if(!object.at<uchar>(y+1,x+1)) fill_space(x+1, y+1,_min,_max);
			if(!object.at<uchar>(y+1,x  )) fill_space(x  , y+1,_min,_max);
			if(!object.at<uchar>(y  ,x-1)) fill_space(x-1, y  ,_min,_max);
			if(!object.at<uchar>(y-1,x-1)) fill_space(x-1, y-1,_min,_max);
			if(!object.at<uchar>(y-1,x  )) fill_space(x  , y-1,_min,_max);
			if(!object.at<uchar>(y-1,x+1)) fill_space(x+1, y-1,_min,_max);
			if(!object.at<uchar>(y+1,x-1)) fill_space(x-1, y+1,_min,_max);

			//if(x > 0 && y > 0 && x+1 < img0.size().width && y < img0.size().height)if(!object.at<uchar>(y  ,x+1)) fill_space(x+1, y  ,_min,_max);
			//if(x > 0 && y > 0 && x+1 < img0.size().width && y+1 < img0.size().height)if(!object.at<uchar>(y+1,x+1)) fill_space(x+1, y+1,_min,_max);
			//if(x > 0 && y > 0 && x < img0.size().width && y+1 < img0.size().height)if(!object.at<uchar>(y+1,x  )) fill_space(x  , y+1,_min,_max);
			//if(x-1 > 0 && y > 0 && x < img0.size().width && y < img0.size().height)if(!object.at<uchar>(y  ,x-1)) fill_space(x-1, y  ,_min,_max);
			//if(x-1 > 0 && y-1 > 0 && x < img0.size().width && y < img0.size().height)if(!object.at<uchar>(y-1,x-1)) fill_space(x-1, y-1,_min,_max);
			//if(x > 0 && y-1 > 0 && x < img0.size().width && y < img0.size().height)if(!object.at<uchar>(y-1,x  )) fill_space(x  , y-1,_min,_max);
			//if(x > 0 && y-1 > 0 && x+1 < img0.size().width && y < img0.size().height)if(!object.at<uchar>(y-1,x+1)) fill_space(x+1, y-1,_min,_max);
			//if(x-1 > 0 && y > 0 && x < img0.size().width && y+1 < img0.size().height)if(!object.at<uchar>(y+1,x-1)) fill_space(x-1, y+1,_min,_max);
		}
	//}
}

static void on_tracker_ground_mouse(int event, int x, int y, int flags, void *void_data){
	if (event != CV_EVENT_LBUTTONUP)
        return;

	click.x = x;
	click.y = y;
	//flag = true;

	//cv::circle(img0, cv::Point(x,y), 5, cv::Scalar(255,255,255,255));
	
	uchar value = img0.at<uchar>(y,x);
	int _min = value - THRESHOLD_RANGE;
	int _max = value + THRESHOLD_RANGE;

	//Create All Black Mask
	object.create(img0.size());
	for(int i = 0 ; i < img0.rows ; i++){
		for(int j = 0 ; j < img0.cols; j++){
			object.at<uchar>(i,j) = 0;
		}
	}

	double t = (double)cv::getTickCount();
	
	fill_space(x,y,_min,_max);
	
	t = (double)cv::getTickCount() - t;
    printf( "execution time = %gms\n", t*1000./cv::getTickFrequency() );

	cv::imshow("result",object);
	cv::waitKey();

}

/*


static void on_tracker_ground_mouse(int event, int x, int y, int flags, void *void_data)
{
    if (event != CV_EVENT_LBUTTONUP)
        return;

	click.x = x;
	click.y = y;
	//flag = true;

	//cv::circle(img0, cv::Point(x,y), 5, cv::Scalar(255,255,255,255));
	
	uchar value = img0.at<uchar>(y,x);

	min = value - THRESHOLD_RANGE;
	max = value + THRESHOLD_RANGE-1;

	//cv::GaussianBlur(img0,img0,cv::Size(3,3),0);

	c_1.create(img0.size());
	c_2.create(img0.size());
	c_3.create(img0.size());
	imgGray_sum.create(img0.size());
	object.create(img0.size());

	for(int i = 0 ; i < img0.rows ; i++){
		for(int j = 0 ; j < img0.cols; j++){
			c_1.at<cv::Vec3b>(i,j)[0] = 255;
			c_1.at<cv::Vec3b>(i,j)[1] = 0;
			c_1.at<cv::Vec3b>(i,j)[2] = 0;

			c_2.at<cv::Vec3b>(i,j)[0] = 0;
			c_2.at<cv::Vec3b>(i,j)[1] = 255;
			c_2.at<cv::Vec3b>(i,j)[2] = 0;

			c_3.at<cv::Vec3b>(i,j)[0] = 0;
			c_3.at<cv::Vec3b>(i,j)[1] = 0;
			c_3.at<cv::Vec3b>(i,j)[2] = 255;

			object.at<uchar>(i,j) = 0;

			if(img0.at<uchar>(i,j) >= min && img0.at<uchar>(i,j) <= max){
				imgGray.at<uchar>(i,j) = img0.at<uchar>(i,j);
			}
			else{
				imgGray.at<uchar>(i,j) = 0;
			}
		}
	}
	//cv::namedWindow("thresh",1);
	//cv::moveWindow("thresh",0,0);
	//cv::namedWindow("old",1);
	//cv::moveWindow("old",640,0);
	//cv::namedWindow("realy_old",1);
	//cv::moveWindow("realy_old",0,480);
	//cv::namedWindow("diff",1);
	//cv::moveWindow("diff",640,480);
	//cv::namedWindow("sum",1);
	//cv::moveWindow("sum",1280+640,0);


	imgGray.copyTo(imgGray_1);
	imgGray.copyTo(imgGray_2);
	imgGray.copyTo(imgGray_3);
	imgGray.copyTo(imgGray_4);
	cv::threshold(imgGray,imgGray_1,0.1,255,CV_THRESH_BINARY);
	cv::threshold(imgGray,imgGray_2,0.1,255,CV_THRESH_BINARY);
	cv::threshold(imgGray,imgGray_3,0.1,255,CV_THRESH_BINARY);
	cv::threshold(imgGray,imgGray_4,0.1,255,CV_THRESH_BINARY);


	while(true){
		for(int i = 0 ; i < img0.rows ; i++){
			for(int j = 0 ; j < img0.cols; j++){
				if(img0.at<uchar>(i,j) > min && img0.at<uchar>(i,j) < max){
					imgGray.at<uchar>(i,j) = img0.at<uchar>(i,j);
				}
				else{
					imgGray.at<uchar>(i,j) = 0;
				}
			}
		}

		cv::threshold(imgGray,imgGray_1,0.1,255,CV_THRESH_BINARY);
		cv::bitwise_and(imgGray_1, imgGray_2,imgGray_diff);
		cv::bitwise_and(imgGray_diff, imgGray_3,imgGray_diff);
		//cv::bitwise_and(imgGray_diff, imgGray_4,imgGray_diff);

		
		
		fill_space(y,x);
		
		
		cv::bitwise_and(imgGray_diff, object,object_diff);

		bool found = false;
		bool aux = false;
		while(!found){
			for(int i = 0 ; i < object_diff.rows ; i++){
				for(int j = 0 ; j < object_diff.cols; j++){
					if(object_diff.at<uchar>(i,j)){
						fill_space(i,j);
						object_diff.at<uchar>(i,j) = 0;
						//break;
						found = true;
						aux = true;
						//cv::imshow("object_diff",object_diff);
						//cv::waitKey();
					}
				}
			}
			found = !found;
		}
		static int aux2 = 0;
		if(!aux){
			if(aux2 <= 0){
				aux2++;
			}
			else{
				if(aux2 == 1){
					aux2++;
				}
				else{
					if(aux2 == 2){
						aux2++;
					}
					else{
						if(aux2 == 3){
							aux2++;
						}
						else{
							fill_flag++;// = !fill_flag;
							aux2 = -2;
						}
					}
				}
			}
		}
		
		//cv::imshow("object_diff",object_diff);


		//cv::waitKey();

//		cv::bitwise_or(imgGray_1, imgGray_2,imgGray_sum);
//		cv::bitwise_or(imgGray_sum, imgGray_3,imgGray_sum);

		//for(int i = 0 ; i < img0.rows ; i++){
		//	for(int j = 0 ; j < img0.cols; j++){
		//		imgGray_sum.at<cv::Vec3b>(i,j)[0] = 0;
		//		imgGray_sum.at<cv::Vec3b>(i,j)[1] = 0;
		//		imgGray_sum.at<cv::Vec3b>(i,j)[2] = 0;

		//		if(imgGray_1.at<uchar>(i,j))
		//			imgGray_sum.at<cv::Vec3b>(i,j)[0] = 255;
		//		if(imgGray_2.at<uchar>(i,j))
		//			imgGray_sum.at<cv::Vec3b>(i,j)[1] = 255;
		//		if(imgGray_3.at<uchar>(i,j))
		//			imgGray_sum.at<cv::Vec3b>(i,j)[2] = 255;
		//		
		//	}
		//}

		////cv::resizeWindow("thresh",320,240);
		//cv::imshow("thresh",imgGray_1);
		//cv::imshow("old",imgGray_2);
		//cv::imshow("realy_old",imgGray_3);
		//cv::imshow("diff",imgGray_diff);
		//cv::imshow("sum",imgGray_sum);
		PRINT_THRESH(min,max)
		char c = cv::waitKey(1);
		cv::imshow("object",object);
		//process_key(c);
		imgGray_3.copyTo(imgGray_4);
		imgGray_2.copyTo(imgGray_3);
		imgGray_1.copyTo(imgGray_2);

		if(turn){
			round -= 10;
			if(round - 10 < 100) turn = false;
		}
		else{
			round += 10;
			if(round + 10 > 255) turn = true;
		}

		if(fill_flag == 1){
			min--;
			max--;
			if(min -1 < 100 )fill_flag = false;
		}
		if(fill_flag == 0){
			min++;
			max++;
			if(max +1 > 210) fill_flag = 1;
		}
		if(fill_flag == 2){
			break;
		}
	}
}
*/

int main(int argc, char*argv[]){
	char* filename = argc >= 2 ? argv[1] : (char*)"dep.png";//"depth_masked1.png";
    img0 = cv::imread(filename, 0);
	imgGray.create(img0.size());

	cv::namedWindow("orig");
	cv::moveWindow("orig",1280,0);
	cv::setMouseCallback("orig", on_tracker_ground_mouse);
	
	while((c = cv::waitKey(30)) != 27){
		process_key(c);

		if(flag){
			cv::imshow("thresh",imgGray);

		}

		cv::imshow("orig",img0);

	}

	exit(0);
}