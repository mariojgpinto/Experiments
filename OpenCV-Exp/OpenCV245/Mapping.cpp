#include "Mapping.h"

#include <opencv2\opencv.hpp>
#include <ToolBoxCV.h>

int img_width = 640*1.5;
int img_height = 480*1.5;

cv::Mat map_original;
cv::Mat map_crop;
cv::Mat map_resize;// = cv::Mat::zeros(cv::Size(img_width,img_height),CV_8UC3);

int center_x;
int center_y;

namespace
{

struct TrackerGroundMouseData
{
    std::string window_name;
    cv::Mat image;
    std::vector<cv::Point> points;
};

static void on_tracker_ground_mouse(int event, int x, int y, int flags, void *void_data)
{
    if (event != CV_EVENT_LBUTTONUP)
        return;

	center_x = x;
	center_y = y;

    //TrackerGroundMouseData* data = (TrackerGroundMouseData*)void_data;
    //data->points.push_back(cv::Point(x, y));
    //circle(data->image, cv::Point(x,y), 3, cv::Scalar(255,0,0,255));
    //imshow(data->window_name, data->image);
}

static void on_tracker_ground_mouse2(int event, int x, int y, int flags, void *void_data)
{
    if (event != CV_EVENT_LBUTTONUP)
        return;

    TrackerGroundMouseData* data = (TrackerGroundMouseData*)void_data;
    data->points.push_back(cv::Point(x, y));
    circle(data->image, cv::Point(x,y), 3, cv::Scalar(255,0,0,255));
    imshow(data->window_name, data->image);
}

}


int main_mapping(int argc, char* argv[]){
	map_original = cv::imread("AAL\\plan_big.png");

	//cv::imshow("Plant Orig",map_original);


	cv::namedWindow("Plant Resized");
	int degrees = 0;
	int size = 100;

	cv::createTrackbar("Degrees","Plant Resized",&degrees,360);
	cv::createTrackbar("Size","Plant Resized",&size,500);


	std::vector<cv::Point> points;
	//points.push_back(cv::Point( 760, 233));
	//points.push_back(cv::Point( 765, 535));
	//points.push_back(cv::Point(1029, 538));
	//points.push_back(cv::Point(1098, 915));
	//points.push_back(cv::Point(1135,1313));
	//points.push_back(cv::Point(2100,1310));
	//points.push_back(cv::Point(2108, 236));

	points.push_back(cv::Point(2219,1610));
	points.push_back(cv::Point(1147,1611));
	points.push_back(cv::Point(1119,2194));
	points.push_back(cv::Point(1030,2748));
	points.push_back(cv::Point( 784,2751));
	points.push_back(cv::Point( 786,2999));
	points.push_back(cv::Point(2220,2999));

	int r_min_x = INT_MAX, r_min_y = INT_MAX;
	int r_max_x = -INT_MAX, r_max_y = -INT_MAX;

	for(int i = 0 ; i < points.size() ; i++){
		if(points[i].x > r_max_x)
			r_max_x = points[i].x;
		if(points[i].x < r_min_x)
			r_min_x = points[i].x;
		if(points[i].y > r_max_y)
			r_max_y = points[i].y;
		if(points[i].y < r_min_y)
			r_min_y = points[i].y;
	}

	int step = 100;
	int map_width = map_original.size().width;
	int map_height = map_original.size().height;
	r_min_x = (r_min_x - step > 0) ? r_min_x - step : 0;
	r_min_y = (r_min_y - step > 0) ? r_min_y - step : 0;
	r_max_x = (r_max_x + 2*step < map_width) ? r_max_x + 2*step : map_width;
	r_max_y = (r_max_y + 2*step < map_height) ? r_max_y + 2*step : map_height;

	cv::Rect roi(r_min_x,r_min_y,r_max_x - r_min_x,r_max_y - r_min_y);//cv::Rect roi(786,1607,1433,1393);

	map_original(roi).copyTo(map_crop);

	//cv::imshow("Plant Cut",map_crop);

	cv::Rect roi_resize;

	double orig_ratio = ToolBoxCV::fitt_image(map_crop,map_resize,img_width,img_height,&roi_resize);
	//map_resize = cv::imread("AAL\\map.png");

	//cv::imwrite("map.png",map_resize);


	cv::setMouseCallback("Plant Resized", on_tracker_ground_mouse);
	cv::RotatedRect rect;


	center_x = 678;
	center_y = 91;
	size = 90;
	degrees = 0;
	center_x = 678;
	center_y = 91;
	size = 90;
	degrees = 0;
	while(cv::waitKey(12) != 27){
		cv::Mat copy;
		map_resize.copyTo(copy);
		rect.center = cv::Point(center_x,center_y);
		rect.size = cv::Size2f(size,size);
		rect.angle = degrees;
		cv::Point2f rect_points[4]; rect.points( rect_points );
		for( int j = 0; j < 4; j++ ){
			char buff[8];
			itoa(j,buff,10);
			cv::putText(copy,buff,cv::Point(rect_points[j].x,rect_points[j].y),CV_FONT_HERSHEY_SIMPLEX,0.8,cvScalar(0,255,0),2);
			line( copy, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,0,255), 3, 8 );
		}
	
		//cv::Mat over = cv::Mat::zeros(cv::Size(img_width,img_height),CV_8UC3);
		//cv::rectangle(over,rect,cv::Scalar(255,0,0,255),-1);

		//cv::line(map_resize,cv::Point(rect.x,rect.y),cv::Point(rect.x,rect.y+rect.height),cv::Scalar(0,0,255),3);
		//cv::line(map_resize,cv::Point(rect.x,rect.y),cv::Point(rect.x+rect.width,rect.y),cv::Scalar(0,0,255),3);
		//cv::line(map_resize,cv::Point(rect.x,rect.y+rect.height),cv::Point(rect.x+rect.width,rect.y+rect.height),cv::Scalar(0,0,255),3);
		//cv::line(map_resize,cv::Point(rect.x+rect.width,rect.y),cv::Point(rect.x+rect.width,rect.y+rect.height),cv::Scalar(0,0,255),3);

		cv::imshow("Plant Resized",copy);
	}

	cv::RotatedRect orig_rect = rect;
	cv::Rect roi_resize_top;
	cv::Mat top_view2 = cv::imread("AAL\\top.png");
	cv::Mat top_view;
	double top_view_ratio = ToolBoxCV::fitt_image(top_view2,top_view,img_width,img_height,&roi_resize_top);

	cv::imwrite("top.png",top_view);

	//cv::setMouseCallback("Plant Resized", on_tracker_ground_mouse);
	center_x = 341;
	center_y = 412;
	size = 145;
	degrees = 46;

	while(cv::waitKey(12) != 27){
		cv::Mat copy;
		top_view.copyTo(copy);
		
		rect.center = cv::Point(center_x,center_y);
		rect.size = cv::Size2f(size,size);
		rect.angle = degrees;

		cv::Point2f rect_points[4]; rect.points( rect_points );
		for( int j = 0; j < 4; j++ ){
			char buff[8];
			itoa(j,buff,10);
			cv::putText(copy,buff,cv::Point(rect_points[j].x,rect_points[j].y),CV_FONT_HERSHEY_SIMPLEX,0.8,cvScalar(0,255,0),2);
			line( copy, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,0,255), 3, 8 );
		}
	
		//cv::Mat over = cv::Mat::zeros(cv::Size(img_width,img_height),CV_8UC3);
		//cv::rectangle(over,rect,cv::Scalar(255,0,0,255),-1);

		//cv::line(map_resize,cv::Point(rect.x,rect.y),cv::Point(rect.x,rect.y+rect.height),cv::Scalar(0,0,255),3);
		//cv::line(map_resize,cv::Point(rect.x,rect.y),cv::Point(rect.x+rect.width,rect.y),cv::Scalar(0,0,255),3);
		//cv::line(map_resize,cv::Point(rect.x,rect.y+rect.height),cv::Point(rect.x+rect.width,rect.y+rect.height),cv::Scalar(0,0,255),3);
		//cv::line(map_resize,cv::Point(rect.x+rect.width,rect.y),cv::Point(rect.x+rect.width,rect.y+rect.height),cv::Scalar(0,0,255),3);

		cv::imshow("Plant Resized",copy);
	}

	cv::RotatedRect top_view_rect = rect;

	printf("");

	cv::Point2f* points_src = (cv::Point2f*)malloc(sizeof(cv::Point2f) * 4);
	cv::Point2f* points_top = (cv::Point2f*)malloc(sizeof(cv::Point2f) * 4);

	orig_rect.points(points_src);
	top_view_rect.points(points_top);

	std::vector<cv::Point> input_original;
	std::vector<cv::Point> input_top_view;

	for(int i = 0 ; i < 4 ; ++i){
		points_src[i].x = ((points_src[i].x - roi_resize.x) / orig_ratio) + roi.x;
		points_src[i].y = ((points_src[i].y - roi_resize.y) / orig_ratio) + roi.y;

		points_top[i].x = ((points_top[i].x - roi_resize_top.x) / top_view_ratio);
		points_top[i].y = ((points_top[i].y - roi_resize_top.y) / top_view_ratio);

		input_original.push_back(cv::Point(	((points_src[i].x - roi_resize.x) * orig_ratio) + roi.x,
											((points_src[i].y - roi_resize.y) * orig_ratio) + roi.y));
		input_top_view.push_back(cv::Point(points_top[i].x,points_top[i].y));
	}

	if(input_original.size() && (input_original.size() == input_top_view.size())){
		cv::Mat trans;
		cv::Mat _mat_perpective;

		trans = cv::getPerspectiveTransform(points_top,points_src);
		cv::warpPerspective(top_view,_mat_perpective,trans,cv::Size(map_resize.cols,map_resize.rows));
		int asd1 = map_resize.type();
		int asd2 = _mat_perpective.type();
		int asd3 = top_view.type();

		int asd4 = CV_16UC1;
		int asd5 = CV_16UC3;
		int asd6 = CV_32FC1;
		int asd7 = CV_32FC3;
		int asd8 = CV_8UC1;
		int asd9 = CV_8UC3;
		int asd10 = CV_64FC1;
		int asd11 = CV_64FC3;

		cv::imshow("perspective",_mat_perpective);

		_mat_perpective += (map_resize*0.5);


		TrackerGroundMouseData ground_mouse_data3;
		ground_mouse_data3.window_name = "top";
		cv::namedWindow(ground_mouse_data3.window_name,1);
		ground_mouse_data3.image = top_view2;
		imshow(ground_mouse_data3.window_name, ground_mouse_data3.image);
		cv::setMouseCallback(ground_mouse_data3.window_name, on_tracker_ground_mouse2, &ground_mouse_data3);


		int old_size = 0;
		while(cv::waitKey(33) != 27){
			if(ground_mouse_data3.points.size() != old_size){
				//cv::circle(_top_view_color,cv::Point(100,100),4,cv::Scalar(0,255,0),-1);
				cv::Point2f pt1(ground_mouse_data3.points.at(ground_mouse_data3.points.size()-1).x,
								ground_mouse_data3.points.at(ground_mouse_data3.points.size()-1).y);
				std::vector<cv::Point2f> obj_src(1);
				std::vector<cv::Point2f> obj_dst(1);
				obj_src[0] = pt1;

				cv::perspectiveTransform(obj_src,obj_dst,trans);

				//float cx = ((obj_dst[0].x - roi_resize.y) / orig_ratio) + roi.x;
				//float cy = ((obj_dst[0].y - roi_resize.x) / orig_ratio) + roi.y;

				cv::circle(_mat_perpective,cv::Point((int)obj_dst[0].x,(int)obj_dst[0].y),4,cv::Scalar(0,255,0),-1);

				old_size = ground_mouse_data3.points.size();
			}
			cv::imshow("top",ground_mouse_data3.image);
			cv::imshow("persp",_mat_perpective);
		}
	}


	//cv::waitKey();

	/*
	cv::Rect roi_resize;
	float ratio;
	int w,h;
	cv::Mat temp;
	
	float coiso2 = ((float)map_crop.size().width/(float)map_crop.size().height);

	bool coiso = coiso2 < (float)img_width/(float)img_height;

	if(coiso){
		ratio = (float)img_height/(float)map_crop.size().height;

		w = map_crop.size().width *ratio;
		h = map_crop.size().height *ratio;

		
		cv::resize(map_crop,temp,cv::Size(w,h),0.0,0.0,cv::INTER_CUBIC);
		
		cv::imshow("Temp",temp);

		roi_resize.x = (img_width - w) / 2.0;
		roi_resize.y = 0;
		roi_resize.width = w;
		roi_resize.height = h;

		temp.copyTo(map_resize(roi_resize));
	}
	else{
		ratio = (float)img_width/(float)map_crop.size().width;

		w = map_crop.size().width *ratio;
		h = map_crop.size().height *ratio;

		cv::resize(map_crop,temp,cv::Size(w,h));
		
		cv::imshow("Temp",temp);

		roi_resize.x = 0;
		roi_resize.y = (img_height - h) / 2.0;
		roi_resize.width = w;
		roi_resize.height = h;

		temp.copyTo(map_resize(roi_resize));
	}

	//cv::resize(map_crop,map_resize,cv::Size(640,480));*/

	//cv::imshow("Plant Resized",map_resize);


	//TrackerGroundMouseData ground_mouse_data;
	//ground_mouse_data.window_name = "Plant Resized";
	//cv::namedWindow(ground_mouse_data.window_name, CV_WINDOW_NORMAL|CV_WINDOW_KEEPRATIO|CV_GUI_EXPANDED);
	//ground_mouse_data.image = map_resize;
	//imshow(ground_mouse_data.window_name, ground_mouse_data.image);
	//cv::setMouseCallback(ground_mouse_data.window_name, on_tracker_ground_mouse, &ground_mouse_data);
	//while (ground_mouse_data.points.size() != 1)
	//{
	//	cv::waitKey(30);
	//}


	//

	//cv::Point pt_orig;
	////pt_orig.x = (ground_mouse_data.points[0].x - roi_resize.x) * ratio + roi.x;
	////pt_orig.y = (ground_mouse_data.points[0].y - roi_resize.y) * ratio + roi.y;
	//pt_orig.x = (ground_mouse_data.points[0].x - roi_resize.x);
	//pt_orig.y = (ground_mouse_data.points[0].y - roi_resize.y);

	////cv::circle(temp,pt_orig,5,cv::Scalar(0,255,0),-1);
	////cv::imshow("Temp",temp);
	////cv::waitKey(0);


	//pt_orig.x /= ratio ;
	//pt_orig.y /= ratio;

	//cv::circle(map_crop,pt_orig,5,cv::Scalar(0,255,0),-1);
	//cv::imshow("Plant Cut",map_crop);
	////cv::waitKey(0);

	//pt_orig.x += roi.x;
	//pt_orig.y += roi.y;
	//cv::circle(map_original,pt_orig,5,cv::Scalar(0,255,0),-1);
	//cv::imshow("Plant Orig",map_original);
	//cv::waitKey(0);

	return 0;
}