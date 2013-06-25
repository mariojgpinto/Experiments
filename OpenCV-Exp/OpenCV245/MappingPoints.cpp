#include "Mapping.h"

#include <opencv2\opencv.hpp>

namespace
{
	struct TrackerGroundMouseData
	{
		std::string window_name;
		cv::Mat3b image;
		std::vector<cv::Point> points;
	};

	static void on_tracker_ground_mouse(int event, int x, int y, int flags, void *void_data)
	{
		if (event != CV_EVENT_LBUTTONUP)
			return;

		TrackerGroundMouseData* data = (TrackerGroundMouseData*)void_data;
		data->points.push_back(cv::Point(x, y));
		circle(data->image, cv::Point(x,y), 3, cv::Scalar(0,255,0,255),-1);
		imshow(data->window_name, data->image);
	}

	static void on_tracker_ground_mouse_numbered(int event, int x, int y, int flags, void *void_data)
	{
		if (event != CV_EVENT_LBUTTONUP)
			return;

		TrackerGroundMouseData* data = (TrackerGroundMouseData*)void_data;
		data->points.push_back(cv::Point(x, y));
		circle(data->image, cv::Point(x,y), 3, cv::Scalar(0,0,255,255),-1);
		char buff[8];
		itoa(data->points.size(),buff,10);
		cv::putText(data->image,buff,cv::Point(x,y),CV_FONT_HERSHEY_SIMPLEX,0.8,cvScalar(0,255,0),2);
		imshow(data->window_name, data->image);
	}
}

void get_user_points(cv::Mat3b map1, std::vector<cv::Point*>* points_out1, cv::Mat3b map2, std::vector<cv::Point*>* points_out2, int n_points){
	if(!points_out1 || !points_out2 || !n_points) return;

	//Estimate ground plane equation asking 10 points to the user.
	TrackerGroundMouseData ground_mouse_data1;
	ground_mouse_data1.window_name = "MAP 1";
	cv::namedWindow(ground_mouse_data1.window_name,1);
	ground_mouse_data1.image = map1;
	imshow(ground_mouse_data1.window_name, ground_mouse_data1.image);
	cv::setMouseCallback(ground_mouse_data1.window_name, on_tracker_ground_mouse_numbered, &ground_mouse_data1);

	while (ground_mouse_data1.points.size() != n_points)
	{
		cv::waitKey(10);
	}

	if(ground_mouse_data1.points.size()){
		for(unsigned int i = 0 ; i < ground_mouse_data1.points.size() ; i++){
			cv::Point* pt = new cv::Point(ground_mouse_data1.points[i]);
			points_out1->push_back(pt);
		}
	}

	
	TrackerGroundMouseData ground_mouse_data2;
	ground_mouse_data2.window_name = "MAP 2";
	cv::namedWindow(ground_mouse_data2.window_name,1);
	ground_mouse_data2.image = map2;
	imshow(ground_mouse_data2.window_name, ground_mouse_data2.image);
	cv::setMouseCallback(ground_mouse_data2.window_name, on_tracker_ground_mouse_numbered, &ground_mouse_data2);

	while (ground_mouse_data2.points.size() != n_points)
	{
		cv::waitKey(10);
	}

	if(ground_mouse_data2.points.size()){
		for(unsigned int i = 0 ; i < ground_mouse_data2.points.size() ; i++){
			cv::Point* pt = new cv::Point(ground_mouse_data2.points[i]);
			points_out2->push_back(pt);
		}
	}

	cv::destroyWindow(ground_mouse_data2.window_name);
	cv::destroyWindow(ground_mouse_data1.window_name);
}

bool get_perspective_transform(cv::Mat& trans_out, std::vector<cv::Point*>* points1, std::vector<cv::Point*>* points2){
	if(!points1 || !points2) return false;

	cv::Point2f* src = (cv::Point2f*)malloc(sizeof(cv::Point2f) * points1->size());
	cv::Point2f* dst = (cv::Point2f*)malloc(sizeof(cv::Point2f) * points2->size());

	for(int i = 0 ; i < points1->size() ; i++){
		src[i] = cv::Point2f((float)points1->at(i)->x,(float)points1->at(i)->y);
		dst[i] = cv::Point2f((float)points2->at(i)->x,(float)points2->at(i)->y);
	}

	trans_out = cv::getPerspectiveTransform(src,dst);

	return true;
}

int main_mapping_points(int argc, char* argv[]){
	cv::Mat3b _original_map = cv::imread("map.png");
	//cv::Mat1b _top_view_1b = cv::imread("object_map.png");
	cv::Mat3b _top_view_3b = cv::imread("top.png");
	cv::Mat _top_view_1b;

	cv::threshold(_top_view_3b,_top_view_1b,0.5,255.0,CV_THRESH_BINARY);

	std::vector<cv::Point*>* _input_original = new std::vector<cv::Point*>();
	std::vector<cv::Point*>* _input_top_view = new std::vector<cv::Point*>();

	get_user_points(_top_view_3b,_input_top_view,_original_map,_input_original,4);

	if(_input_original->size() && (_input_original->size() == _input_top_view->size())){
		cv::Mat trans;

		if(!get_perspective_transform(trans,_input_top_view,_input_original)) return false;

		cv::Mat _mat_perpective;

		cv::warpPerspective(_top_view_3b,_mat_perpective,trans,cv::Size(_original_map.cols,_original_map.rows));


		_mat_perpective += (_original_map*0.5);


		TrackerGroundMouseData ground_mouse_data3;
		ground_mouse_data3.window_name = "top";
		cv::namedWindow(ground_mouse_data3.window_name,1);
		ground_mouse_data3.image = _top_view_3b;
		imshow(ground_mouse_data3.window_name, ground_mouse_data3.image);
		cv::setMouseCallback(ground_mouse_data3.window_name, on_tracker_ground_mouse, &ground_mouse_data3);


		int old_size = 0;
		while(cv::waitKey(33) != 27){
			if(ground_mouse_data3.points.size() != old_size){
				//cv::circle(_top_view_color,cv::Point(100,100),4,cv::Scalar(0,255,0),-1);
				cv::Point2f pt1(ground_mouse_data3.points.at(ground_mouse_data3.points.size()-1).x,
								ground_mouse_data3.points.at(ground_mouse_data3.points.size()-1).y);
				std::vector<cv::Point2f> obj_srt(1);
				std::vector<cv::Point2f> obj_dst(1);
				obj_srt[0] = pt1;

				cv::perspectiveTransform(obj_srt,obj_dst,trans);
				cv::circle(_mat_perpective,cv::Point((int)obj_dst[0].x,(int)obj_dst[0].y),4,cv::Scalar(0,255,0),-1);
			}
			cv::imshow("top",ground_mouse_data3.image);
			cv::imshow("persp",_mat_perpective);
		}
	}

	cv::imshow("_top_view_1b",_top_view_1b);

	cv::waitKey();




	return 0;
}