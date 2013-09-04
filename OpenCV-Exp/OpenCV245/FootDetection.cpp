#include "FootDetection.h"

#include <opencv2\opencv.hpp>

int main_foot_detection(int argc, char* argv[]){
	cv::Mat image_orig = cv::imread("Walkys\\Foots\\imageR_0.png",0);
	cv::Mat image = cv::imread("Walkys\\Foots\\imageR_0.png",0);

	cv::Mat top_countours = cv::Mat::zeros(image.size(),CV_8UC3);
	cv::vector<cv::vector<cv::Point> > contours;
	cv::vector<cv::Vec4i> hierarchy;

	cv::findContours( image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

	if(contours.size() < 1) return -1;

	cv::Rect rect;

	for(unsigned int i = 0; i < contours.size(); i++ ){
		rect = cv::boundingRect(contours[i]);
		drawContours( top_countours , contours, i, cv::Scalar(255,128,128), -1, 1, hierarchy, 0, cv::Point() );
		cv::rectangle(top_countours , rect , cv::Scalar(0,255,0), 1);
	}

	uchar* ptr_mask = image_orig.data;

	int idx = 0;

	int min_left = 900;

	int* lefts = (int*)malloc(sizeof(int) * rect.height);
	int* rights = (int*)malloc(sizeof(int) * rect.height);
	
	
	int top_y;

	for(int y = rect.y + rect.height ; y > rect.y; --y){
		bool left = false;
		int left_x = 0;
		bool right = false;
		int right_x = 0;
		
		for(int x = rect.x ; x < rect.x + rect.width && !left ; ++x){
			if(ptr_mask[y * 640 + x]){
				left = true;
				left_x = x;
			}	
		}

		for(int x = rect.x + rect.width ; x > rect.x && !right ; --x){
			if(ptr_mask[y * 640 + x]){
				right = true;
				right_x = x;
			}	
		}

		if(left && right){
			lefts[idx] = left_x;
			rights[idx] = right_x;

			if(left_x < min_left){
				min_left = left_x;
			}
			else{
				if(left_x > (min_left + 5)){
					top_y = y;
					goto end;
				}
			}

			cv::circle(top_countours,cv::Point(left_x, y), 2,cv::Scalar(255,0,0));
			cv::circle(top_countours,cv::Point(right_x, y), 2,cv::Scalar(255,0,0));

			cv::imshow("top_countours",top_countours);

			printf("%d Dist %d | X:%d\n", idx++, right_x - left_x, left_x);

			cv::waitKey();
		}
	}

	idx = 0;
	for(int y = rect.y + rect.height ; y > rect.y; --y){
		printf("C:%d , idx:%d - %d\n",lefts[idx] - lefts[idx+10],lefts[idx],lefts[idx+10]);
		if(lefts[idx] - lefts[idx+10] < 10){
			idx += 10;
			top_y = y - 10;
			min_left = lefts[idx - 5];
			break;
		}
		idx++;
	}
//
//	int ac = 10;
//	for(int y = rect.y + rect.height -10 ; y > (rect.y + 10) ; --y , ac++){
//		//cv::circle(top_countours,cv::Point(lefts[ac], y), 2,cv::Scalar(255,0,0));
//		//cv::circle(top_countours,cv::Point(lefts[ac + 10], y - 10), 2,cv::Scalar(255,0,0));
//		//cv::imshow("top_countours",top_countours);
//		printf("Click %d\n",lefts[ac] - lefts[ac + 10]);
////		cv::waitKey();
//		
//		if(lefts[ac] - lefts[ac + 10] < 10){
//			idx = ac + 20;
//			max_y = y - 20;
//			min_left = lefts[ac];
//			cv::circle(top_countours,cv::Point(lefts[idx], max_y), 3,cv::Scalar(0,255,255),-1);
//			cv::circle(top_countours,cv::Point(lefts[ac], y), 3,cv::Scalar(0,255,255),-1);
//			break;
//		}
//	}

	//max_y = rect.y + rect.height / 2;
	//idx = rect.height / 2 - 2;
	//for(int x = rect.x ; x < rect.x + rect.width ; ++x){
	//	if(ptr_mask[max_y * 640 + x]){
	//		min_left = x;
	//		break;
	//	}	
	//}

	//cv::circle(top_countours,cv::Point(lefts[idx], max_y), 5,cv::Scalar(0,255,0),-1);
	//cv::circle(top_countours,cv::Point(lefts[idx - 10], max_y + 10), 5,cv::Scalar(0,255,0),-1);
	//cv::circle(top_countours,cv::Point(lefts[idx + 10], max_y - 10), 5,cv::Scalar(0,255,0),-1);

	//cv::circle(top_countours,cv::Point(lefts[idx], max_y), 15,cv::Scalar(0,0,255),-1);

	//cv::waitKey(13);
	//cv::waitKey();

end:

	idx--;
	int old_idx = idx;
	int bottom_y = -1;

	for(int y = top_y ; y < rect.y + rect.height ; ++y){

		if(lefts[idx] > (min_left + 20)){
			bottom_y = y;
			break;
		}

		idx--;
	}

	int middle_y =  (bottom_y + top_y) / 2;

	bool left = false;
	int left_x = 0;
	bool right = false;
	int right_x = 0;
	for(int x = rect.x ; x < rect.x + rect.width && !left ; ++x){
		if(ptr_mask[middle_y * 640 + x]){
			left = true;
			left_x = x;
		}	
	}

	for(int x = rect.x + rect.width ; x > rect.x && !right ; --x){
	if(ptr_mask[middle_y * 640 + x]){
		right = true;
		right_x = x;
	}	
	}

	int middle_x = left_x + ((right_x - left_x) / 4);

	cv::circle(top_countours,cv::Point(lefts[old_idx], top_y), 5,cv::Scalar(255,0,0),-1);//TOP
	cv::circle(top_countours,cv::Point(lefts[idx], bottom_y), 5,cv::Scalar(0,255,0),-1); //BOTTOM
	cv::circle(top_countours,cv::Point(middle_x , middle_y), 5,cv::Scalar(0,0,255),-1); //MIDDLE
	





	//
	//
	//int old_x = lefts[idx];
	//int old_y = max_y;
	//idx--;
	//int counter = 0;
	//int counter2 = 0;
	//int counter3 = 0;
	//for(int y = max_y ; y < rect.y + rect.height ; ++y){
	//	counter++;
	//	if(counter2 < lefts[idx]){
	//		counter2 = lefts[idx];
	//	}

	//	cv::circle(top_countours,cv::Point(lefts[idx], y), 2,cv::Scalar(255,0,0),-1);

	//	if(lefts[counter] > (min_left + 10)){
	//		counter3 = y;
	//		break;
	//	}

	//	idx--;
	//}
	cv::imshow("top_countours",top_countours);

	cv::waitKey();

	//counter2 /= counter ;
	//counter3 /= counter ;

	//cv::circle(top_countours,cv::Point(counter2, counter3), 3,cv::Scalar(0,0,255),-1);
	//cv::circle(top_countours,cv::Point(counter2 /*+ (counter2 - min_left)*/, (counter3 + old_y) / 2), 5,cv::Scalar(0,0,255), -1);
	
	
	cv::imshow("top_countours",top_countours);
	cv::waitKey();

	return 0;
}


/*
	int clr = 0;

	int max_right = 0;

	int* rights = (int*)malloc(sizeof(int) * rect.height);
	int max_y;

	for(int y = rect.y + rect.height ; y > rect.y; --y){
		bool left = false;
		int left_x = 0;
		bool right = false;
		int right_x = 0;
		
		for(int x = rect.x ; x < rect.x + rect.width && !left ; ++x){
			if(ptr_mask[y * 640 + x]){
				left = true;
				left_x = x;
			}	
		}

		for(int x = rect.x + rect.width ; x > rect.x && !right ; --x){
			if(ptr_mask[y * 640 + x]){
				right = true;
				right_x = x;
			}	
		}

		if(left && right){
			rights[clr] = right_x;

			if(right_x > max_right){
				max_right = right_x;
			}
			else{
				if(right_x < (max_right - 10)){
					max_y = y;
					goto end;
				}
			}

			cv::circle(top_countours,cv::Point(left_x, y), 2,cv::Scalar(255,0,0));
			cv::circle(top_countours,cv::Point(right_x, y), 2,cv::Scalar(255,0,0));

			//cv::imshow("top_countours",top_countours);

			printf("%d Dist %d | X:%d\n", clr++, right_x - left_x, right_x);

			//cv::waitKey();
		}
	}

end:
	
	clr--;
	int old_x = rights[clr];
	int old_y = max_y;
	clr--;
	int counter = 0;
	int counter2 = 0;
	int counter3 = 0;
	for(int y = max_y ; y < rect.y + rect.height ; ++y){
		counter++;
		counter2 += rights[clr];
		counter3 += y;

		//cv::circle(top_countours,cv::Point(rights[clr], y), 5,cv::Scalar(0,0,255));
		//cv::circle(top_countours,cv::Point(rights[clr], y), 5,cv::Scalar(0,0,255));

		if(rights[clr] < (max_right - 10)){
			break;
		}

		clr--;
	}


	counter2 /= counter ;
	counter3 /= counter ;

	cv::circle(top_countours,cv::Point(counter2, counter3), 3,cv::Scalar(0,0,255),-1);
	//cv::circle(top_countours,cv::Point((old_x + counter2) / 2, (old_y + counter3) / 2), 5,cv::Scalar(0,0,255));
*/