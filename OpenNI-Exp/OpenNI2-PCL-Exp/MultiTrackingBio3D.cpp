#include "NIUser3D.h"

#include "NIViewer.h"

#include <NIKinect2Manager.h>
#include <ToolBox.h>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

const int BODY_PART_HD = 0;

namespace{


class Histograms{
	public:
		Histograms():_hist_rgb(std::vector<float>(9)),_hist_hsv(std::vector<float>(9)){}
		~Histograms(){}

		cv::Mat _histogram_rgb;

		cv::Mat _histogram_rgb_r;
		cv::Mat _histogram_rgb_g;
		cv::Mat _histogram_rgb_b;
		cv::Mat _histogram_rgb_head_r;
		cv::Mat _histogram_rgb_head_g;
		cv::Mat _histogram_rgb_head_b;
		cv::Mat _histogram_rgb_torso_r;
		cv::Mat _histogram_rgb_torso_g;
		cv::Mat _histogram_rgb_torso_b;
		cv::Mat _histogram_rgb_legs_r;
		cv::Mat _histogram_rgb_legs_g;
		cv::Mat _histogram_rgb_legs_b;

		cv::Mat _histogram_rgb_vis;
		cv::Mat _histogram_rgb_vis_full;
		cv::Mat _histogram_rgb_vis_head;
		cv::Mat _histogram_rgb_vis_torso;
		cv::Mat _histogram_rgb_vis_legs;


		cv::Mat _hsv_user;

		cv::Mat _histogram_hsv_h;
		cv::Mat _histogram_hsv_s;
		cv::Mat _histogram_hsv_v;

		cv::Mat _histogram_hsv_full;
		cv::Mat _histogram_hsv_head;
		cv::Mat _histogram_hsv_torso;
		cv::Mat _histogram_hsv_legs;

		cv::Mat _histogram_hsv_full_h;
		cv::Mat _histogram_hsv_full_s;
		cv::Mat _histogram_hsv_full_v;
		cv::Mat _histogram_hsv_head_h;
		cv::Mat _histogram_hsv_head_s;
		cv::Mat _histogram_hsv_head_v;
		cv::Mat _histogram_hsv_torso_h;
		cv::Mat _histogram_hsv_torso_s;
		cv::Mat _histogram_hsv_torso_v;
		cv::Mat _histogram_hsv_legs_h;
		cv::Mat _histogram_hsv_legs_s;
		cv::Mat _histogram_hsv_legs_v;

		cv::Mat _histogram_hsv_vis_full;
		cv::Mat _histogram_hsv_vis_head;
		cv::Mat _histogram_hsv_vis_torso;
		cv::Mat _histogram_hsv_vis_legs;

		cv::Mat _hsv_hist_h;
		cv::Mat _hsv_hist_s;
		cv::Mat _hsv_hist_v;

		cv::Mat _pseudo_h;
		cv::Mat _pseudo_s;

		std::vector<float> _hist_rgb;
		std::vector<float> _hist_hsv;
		int _counter;
		cv::Mat _hist_vis_rgb;
		cv::Mat _hist_vis_hsv;
		cv::Mat _hist_vis_hsv_cvt;
		cv::Mat _hist_img_result_rgb;
		cv::Mat _hist_img_result_hsv;
		cv::Mat _hist_img_result_hsv_cvt;
};

class Features{
	public:
		Features():	_hog_hist_bbox(cv::vector<float>(10)),
					_hog_hist_mask(cv::vector<float>(10)),
					_hog_hist_torso(cv::vector<float>(10)){}
		~Features(){}

		cv::Mat _gray_full;
		cv::Mat _gray_user;
		cv::Mat _gray_user_mask;

		cv::Mat _hog_vis_bbox;
		cv::Mat _hog_vis_mask;

		cv::vector<float> _hog_hist_bbox;
		cv::vector<float> _hog_hist_mask;
		cv::Mat _hog_vis_hist_bbox;
		cv::Mat _hog_vis_hist_mask;
		
		
		cv::Rect _rect_torso;
		cv::Mat _gray_torso;
		cv::Rect _rect_torso_roi;
		cv::Mat _hog_vis_torso;
		cv::vector<float> _hog_hist_torso;
		cv::Mat _hog_vis_hist_torso;

		cv::Mat _canny_torso;
};

class Division3D{
	public:
		Division3D(){
			_cloud.width = 640*480;
			_cloud.height = 1;
			_cloud.points.resize (_cloud.width * _cloud.height);

			_cloud_head.width = 320*240;
			_cloud_head.height = 1;
			_cloud_head.points.resize (_cloud_head.width * _cloud_head.height);

			_cloud_torso.width = 320*240;
			_cloud_torso.height = 1;
			_cloud_torso.points.resize (_cloud_torso.width * _cloud_torso.height);

			_cloud_legs.width = 320*240;
			_cloud_legs.height = 1;
			_cloud_legs.points.resize (_cloud_legs.width * _cloud_legs.height);

			_distances = (float*)malloc(sizeof(float) * _cloud.width * _cloud.height);
			this->_idx_3d_to_2d = (int**)malloc(sizeof(int*) * _cloud.width * _cloud.height);
			for(int i = 0 ; i < _cloud.width * _cloud.height ; ++i){
				this->_idx_3d_to_2d[i] = (int*)malloc(sizeof(int)*2);
				this->_idx_3d_to_2d[i][0] = 0;
				this->_idx_3d_to_2d[i][1] = 0;
			}
			this->_idx_head = (int*)malloc(sizeof(int) * _cloud.width * _cloud.height);;
			this->_idx_torso = (int*)malloc(sizeof(int) * _cloud.width * _cloud.height);;
			this->_idx_legs = (int*)malloc(sizeof(int) * _cloud.width * _cloud.height);;

			_mask_head = cv::Mat::zeros(480,640,CV_8UC1);
			_mask_torso = cv::Mat::zeros(480,640,CV_8UC1);
			_mask_legs = cv::Mat::zeros(480,640,CV_8UC1);
		}
		~Division3D(){}

		pcl::PointCloud<pcl::PointXYZRGB> _cloud;
		int _n_points;
		int **_idx_3d_to_2d;

		pcl::PointCloud<pcl::PointXYZRGB> _cloud_head;
		int _n_points_head;
		int *_idx_head;
		cv::Mat _mask_head_orig;
		cv::Mat _mask_head;

		pcl::PointCloud<pcl::PointXYZRGB> _cloud_torso;
		int _n_points_torso;
		int *_idx_torso;
		cv::Mat _mask_torso_orig;
		cv::Mat _mask_torso;
		
		pcl::PointCloud<pcl::PointXYZRGB> _cloud_legs;
		int _n_points_legs;
		int *_idx_legs;
		cv::Mat _mask_legs_orig;
		cv::Mat _mask_legs;

		float* _distances;

		float _max_height;
		float _min_height;

		float _head_max;
		float _head_min;

		float _torso_max;
		float _torso_min;

		float _leg_max;
		float _leg_min;

};

class User{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		User(){}
		User(User* user){
			this->_bbox = user->_bbox;
			this->_centroid_3d = user->_centroid_3d;
			this->_centroid_2d = user->_centroid_2d;

			user->_color_full.copyTo(this->_color_full);
			user->_mask_full.copyTo(this->_mask_full);
		
			user->_color_user.copyTo(this->_color_user);
			user->_mask_user.copyTo(this->_mask_user);

			this->_id = user->_id;
			this->_conf = user->_conf;
		}
		~User(){}

		cv::Rect	_bbox;
		cv::Point3f	_centroid_3d;
		cv::Point	_centroid_2d;

		cv::Mat _color_full;
		cv::Mat _mask_full;
		cv::Mat _mask_orig;
		
		cv::Mat _color_user;
		cv::Mat _mask_user;
		
		Histograms _histogram;
		Features _features;
		Division3D _division3d;

		NIKinect2* _kinect;
		int _id;
		double _conf;
};

void init_user(User* user, NIKinect2* kinect, nite::UserData* data, cv::Mat color){
	if(!user || !kinect || !data || !color.rows || !color.cols) return;
	
	color.copyTo(user->_color_full);

	user->_kinect = kinect;

	const nite::BoundingBox bbox_nite = data->getBoundingBox();

	user->_bbox.x = bbox_nite.min.x;
	user->_bbox.y = bbox_nite.min.y;
	user->_bbox.width = bbox_nite.max.x - bbox_nite.min.x;
	user->_bbox.height = bbox_nite.max.y - bbox_nite.min.y;

	nite::Point3f world_pt = data->getCenterOfMass();
	user->_centroid_3d.x = world_pt.x;
	user->_centroid_3d.y = world_pt.y;
	user->_centroid_3d.z = world_pt.z;

	float xx,yy,zz;

	openni::CoordinateConverter::convertWorldToDepth(*kinect->get_depth_stream(),
													 world_pt.x, world_pt.y, world_pt.z, 
													 &xx,&yy,&zz);

	user->_centroid_2d.x = xx;
	user->_centroid_2d.y = yy;

	kinect->get_user_mask(data->getId(),user->_mask_orig);
	
	if (data->isNew())
		printf("\n\nNEW USER\n\n");
	

	if(BODY_PART_HD){
		user->_bbox.x *= 2;
		user->_bbox.y *= 2;
		user->_bbox.width *= 2;
		user->_bbox.height *= 2;

		user->_centroid_2d.x *= 2;
		user->_centroid_2d.y *= 2;

		cv::resize(user->_mask_orig,user->_mask_full,cv::Size(1280,960));
	}
	else{
		user->_mask_orig.copyTo(user->_mask_full);
	}

	cv::dilate(user->_mask_full,user->_mask_full,cv::Mat(3,3,CV_8UC1));
	cv::erode(user->_mask_full,user->_mask_full,cv::Mat(3,3,CV_8UC1));

	color(user->_bbox).copyTo(user->_color_user);
	user->_mask_full(user->_bbox).copyTo(user->_mask_user);
}

void generate_3d(User* user){
	if(!user) return;
	
	//cv::Mat depth16U;
	//user->_kinect->get_depth_16(depth16U);
	//uint16_t* ptr = (uint16_t*)depth16U.data;
	uint16_t* ptr = (uint16_t*)user->_kinect->get_depth_frame_ref()->getData();
	openni::RGB888Pixel* ptr_clr = (openni::RGB888Pixel*)user->_kinect->get_color_frame_ref()->getData();
	openni::VideoStream *depthStream = user->_kinect->get_depth_stream();
			
	uchar* mask_ptr = (uchar*)user->_mask_orig.data;

	user->_division3d._cloud.clear();

	int idx = 0;
	int idx_hd = 0;
	user->_division3d._n_points = 0;

	int y_init = (BODY_PART_HD) ? user->_bbox.y/2 : user->_bbox.y;
	int y_end = (BODY_PART_HD) ? user->_bbox.y/2 + user->_bbox.height/2 : user->_bbox.y + user->_bbox.height;
	int x_init = (BODY_PART_HD) ? user->_bbox.x/2 : user->_bbox.x;
	int x_end = (BODY_PART_HD) ? user->_bbox.x/2 + user->_bbox.width/2 : user->_bbox.x + user->_bbox.width;

	for(int y = y_init ; y < y_end ; ++y){
		for(int x = x_init ; x < x_end ; ++x){
			idx =y * 640 + x;
			if(mask_ptr[idx]){
				float d_x = x;
				float d_y = y;
				float d_z = ptr[idx];
				float w_x = 0;
				float w_y = 0;
				float w_z = 0;

				if(d_z > 0){
					openni::CoordinateConverter::convertDepthToWorld(*depthStream,d_x,d_y,d_z,&w_x,&w_y,&w_z);

					pcl::PointXYZRGB pt(ptr_clr[idx].r,
										ptr_clr[idx].g,
										ptr_clr[idx].b);
					if(BODY_PART_HD){
						idx_hd =y*2 * 1280 + x*2;
						pt.r = ptr_clr[idx_hd].r;
						pt.g = ptr_clr[idx_hd].g;
						pt.b = ptr_clr[idx_hd].b;
					}

					pt.x = w_x;
					pt.y = w_y;
					pt.z = w_z;

					user->_division3d._idx_3d_to_2d[user->_division3d._n_points][0] = x;
					user->_division3d._idx_3d_to_2d[user->_division3d._n_points][1] = y;
					user->_division3d._n_points++;
					user->_division3d._cloud.push_back(pcl::PointXYZRGB(pt));
				}
			}
		}
	}
}

void generate_user_parts(User* user, ToolBox::Plane* plane, int head_thresh){
	if(!user || !plane) return;

	if(user->_division3d._cloud.size()){
		user->_division3d._max_height = -INT_MAX;
		user->_division3d._min_height = INT_MAX;
		
		for(int i = 0 ; i < user->_division3d._cloud.size() ; ++i){
			user->_division3d._distances[i] = plane->distance_to_plane( user->_division3d._cloud[i].x,
																		user->_division3d._cloud[i].y,
																		user->_division3d._cloud[i].z);

			if(user->_division3d._distances[i] > user->_division3d._max_height){
				user->_division3d._max_height = user->_division3d._distances[i];
			}

			if(user->_division3d._distances[i] < user->_division3d._min_height){
				user->_division3d._min_height = user->_division3d._distances[i];
			}
		}

		user->_division3d._cloud_head.clear();
		user->_division3d._cloud_torso.clear();
		user->_division3d._cloud_legs.clear();

		user->_division3d._head_max = user->_division3d._max_height;
		user->_division3d._head_min = user->_division3d._max_height - head_thresh;

		user->_division3d._torso_max = user->_division3d._head_min;
		user->_division3d._torso_min = (user->_division3d._head_min) / 1.70;
			
		user->_division3d._leg_max = user->_division3d._torso_min;
		user->_division3d._leg_min = user->_division3d._min_height;

		user->_division3d._n_points_head = 0;
		user->_division3d._n_points_torso = 0;
		user->_division3d._n_points_legs= 0;

		user->_division3d._mask_head_orig = cv::Mat::zeros(480,640,CV_8UC1);
		user->_division3d._mask_torso_orig = cv::Mat::zeros(480,640,CV_8UC1);
		user->_division3d._mask_legs_orig = cv::Mat::zeros(480,640,CV_8UC1);

		uchar* ptr_mask_head = (uchar*)user->_division3d._mask_head_orig.data;
		uchar* ptr_mask_torso = (uchar*)user->_division3d._mask_torso_orig.data;
		uchar* ptr_mask_legs = (uchar*)user->_division3d._mask_legs_orig.data;

		for(int i = 0 ; i < user->_division3d._cloud.size() ; ++i){
			if(user->_division3d._distances[i] > user->_division3d._head_min){ // && user->_distances[i] < user->_head_max
				ptr_mask_head[user->_division3d._idx_3d_to_2d[i][1] * 640 + user->_division3d._idx_3d_to_2d[i][0]] = 255;
				user->_division3d._idx_head[user->_division3d._n_points_head++] = i;
				user->_division3d._cloud_head.push_back(user->_division3d._cloud[i]);
				user->_division3d._cloud_head[user->_division3d._cloud_head.size()-1].r = 255;
				user->_division3d._cloud_head[user->_division3d._cloud_head.size()-1].g = 0;
				user->_division3d._cloud_head[user->_division3d._cloud_head.size()-1].b = 0;
			}

			if(user->_division3d._distances[i] > user->_division3d._torso_min && user->_division3d._distances[i] < user->_division3d._torso_max){
				ptr_mask_torso[user->_division3d._idx_3d_to_2d[i][1] * 640 + user->_division3d._idx_3d_to_2d[i][0]] = 255;
				user->_division3d._idx_torso[user->_division3d._n_points_torso++] = i;
				user->_division3d._cloud_torso.push_back(user->_division3d._cloud[i]);
				user->_division3d._cloud_torso[user->_division3d._cloud_torso.size()-1].r = 0;
				user->_division3d._cloud_torso[user->_division3d._cloud_torso.size()-1].g = 255;
				user->_division3d._cloud_torso[user->_division3d._cloud_torso.size()-1].b = 0;
			}

			if(user->_division3d._distances[i] < user->_division3d._leg_max){ // && user->_distances[i] > user->_head_min
				ptr_mask_legs[user->_division3d._idx_3d_to_2d[i][1] * 640 + user->_division3d._idx_3d_to_2d[i][0]] = 255;
				user->_division3d._idx_legs[user->_division3d._n_points_legs++] = i;
				user->_division3d._cloud_legs.push_back(user->_division3d._cloud[i]);
				user->_division3d._cloud_legs[user->_division3d._cloud_legs.size()-1].r = 0;
				user->_division3d._cloud_legs[user->_division3d._cloud_legs.size()-1].g = 0;
				user->_division3d._cloud_legs[user->_division3d._cloud_legs.size()-1].b = 255;
			}
		}

		if(BODY_PART_HD){
			cv::resize(user->_division3d._mask_head_orig,user->_division3d._mask_head,cv::Size(1280,960));
			cv::resize(user->_division3d._mask_torso_orig,user->_division3d._mask_torso,cv::Size(1280,960));
			cv::resize(user->_division3d._mask_legs_orig,user->_division3d._mask_legs,cv::Size(1280,960));
		}
		else{
			user->_division3d._mask_head_orig.copyTo(user->_division3d._mask_head);
			user->_division3d._mask_torso_orig.copyTo(user->_division3d._mask_torso);
			user->_division3d._mask_legs_orig.copyTo(user->_division3d._mask_legs);
		}
	}
}

void rgb_histogram(User* user){
	// Separate the image in 3 places ( B, G and R )
	cv::vector<cv::Mat> bgr_planes;
	cv::split( user->_color_full, bgr_planes );

	/// Establish the number of bins
	int histSize = 6;
	int histSizeRGB[] = {histSize, histSize, histSize};
	/// Set the ranges ( for B,G,R) )
	float range[] = { 0, 256 } ;
	const float* histRange = { range };
	const float* histRangeRGB[] = { range ,range ,range };

	bool uniform = true; bool accumulate = false;
	int channels[] = { 0, 1 ,2};
	//cv::Mat b_hist, g_hist, r_hist;

	// Draw the histograms for B, G and R
	int hist_w = 256; int hist_h = 200;
	int bin_w = cvRound( (double) hist_w/histSize );


	/// Compute the histograms:
	cv::calcHist( &user->_color_full, 1, channels, user->_mask_full, user->_histogram._histogram_rgb, 2, histSizeRGB, histRangeRGB, uniform, accumulate );
	cv::normalize(user->_histogram._histogram_rgb, user->_histogram._histogram_rgb, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat() );

	cv::calcHist( &bgr_planes[0], 1, 0, user->_mask_full, user->_histogram._histogram_rgb_b, 1, &histSize, &histRange, uniform, accumulate );
	cv::calcHist( &bgr_planes[1], 1, 0, user->_mask_full, user->_histogram._histogram_rgb_g, 1, &histSize, &histRange, uniform, accumulate );
	cv::calcHist( &bgr_planes[2], 1, 0, user->_mask_full, user->_histogram._histogram_rgb_r, 1, &histSize, &histRange, uniform, accumulate );
	/// Normalize the result to [ 0, histImage.rows ]
	cv::normalize(user->_histogram._histogram_rgb_b, user->_histogram._histogram_rgb_b, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(user->_histogram._histogram_rgb_g, user->_histogram._histogram_rgb_g, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(user->_histogram._histogram_rgb_r, user->_histogram._histogram_rgb_r, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat() );
	
	/// HEAD
	cv::calcHist( &bgr_planes[0], 1, 0, user->_division3d._mask_head, user->_histogram._histogram_rgb_head_b, 1, &histSize, &histRange, uniform, accumulate );
	cv::calcHist( &bgr_planes[1], 1, 0, user->_division3d._mask_head, user->_histogram._histogram_rgb_head_g, 1, &histSize, &histRange, uniform, accumulate );
	cv::calcHist( &bgr_planes[2], 1, 0, user->_division3d._mask_head, user->_histogram._histogram_rgb_head_r, 1, &histSize, &histRange, uniform, accumulate );
	
	cv::normalize(user->_histogram._histogram_rgb_head_b, user->_histogram._histogram_rgb_head_b, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(user->_histogram._histogram_rgb_head_g, user->_histogram._histogram_rgb_head_g, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(user->_histogram._histogram_rgb_head_r, user->_histogram._histogram_rgb_head_r, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat() );
	
	/// TORSO
	cv::calcHist( &bgr_planes[0], 1, 0, user->_division3d._mask_torso, user->_histogram._histogram_rgb_torso_b, 1, &histSize, &histRange, uniform, accumulate );
	cv::calcHist( &bgr_planes[1], 1, 0, user->_division3d._mask_torso, user->_histogram._histogram_rgb_torso_g, 1, &histSize, &histRange, uniform, accumulate );
	cv::calcHist( &bgr_planes[2], 1, 0, user->_division3d._mask_torso, user->_histogram._histogram_rgb_torso_r, 1, &histSize, &histRange, uniform, accumulate );
	
	cv::normalize(user->_histogram._histogram_rgb_torso_b, user->_histogram._histogram_rgb_torso_b, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(user->_histogram._histogram_rgb_torso_g, user->_histogram._histogram_rgb_torso_g, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(user->_histogram._histogram_rgb_torso_r, user->_histogram._histogram_rgb_torso_r, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat() );
	
	/// LEGS
	cv::calcHist( &bgr_planes[0], 1, 0, user->_division3d._mask_legs, user->_histogram._histogram_rgb_legs_b, 1, &histSize, &histRange, uniform, accumulate );
	cv::calcHist( &bgr_planes[1], 1, 0, user->_division3d._mask_legs, user->_histogram._histogram_rgb_legs_g, 1, &histSize, &histRange, uniform, accumulate );
	cv::calcHist( &bgr_planes[2], 1, 0, user->_division3d._mask_legs, user->_histogram._histogram_rgb_legs_r, 1, &histSize, &histRange, uniform, accumulate );
	
	cv::normalize(user->_histogram._histogram_rgb_legs_b, user->_histogram._histogram_rgb_legs_b, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(user->_histogram._histogram_rgb_legs_g, user->_histogram._histogram_rgb_legs_g, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(user->_histogram._histogram_rgb_legs_r, user->_histogram._histogram_rgb_legs_r, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat() );
	
	
	user->_histogram._histogram_rgb_vis_full = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
	user->_histogram._histogram_rgb_vis_head = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
	user->_histogram._histogram_rgb_vis_torso= cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
	user->_histogram._histogram_rgb_vis_legs = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

	///// Draw for each channel
	for( int i = 1; i < histSize; i++ )
	{
		cv::line(	user->_histogram._histogram_rgb_vis_full, 
					cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram._histogram_rgb_b.at<float>(i-1)) ) ,
					cv::Point( bin_w*(i  ), hist_h - cvRound(user->_histogram._histogram_rgb_b.at<float>(i  )) ),
					cv::Scalar( 255, 0, 0), 2, 8, 0  );
		cv::line(	user->_histogram._histogram_rgb_vis_full, 
					cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram._histogram_rgb_g.at<float>(i-1)) ) ,
					cv::Point( bin_w*(i  ), hist_h - cvRound(user->_histogram._histogram_rgb_g.at<float>(i  )) ),
					cv::Scalar( 0, 255, 0), 2, 8, 0  );
		cv::line(	user->_histogram._histogram_rgb_vis_full, 
					cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram._histogram_rgb_r.at<float>(i-1)) ) ,
					cv::Point( bin_w*(i  ), hist_h - cvRound(user->_histogram._histogram_rgb_r.at<float>(i  )) ),
					cv::Scalar( 0, 0, 255), 2, 8, 0  );

		//HEAD
		cv::line(	user->_histogram._histogram_rgb_vis_head, 
					cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram._histogram_rgb_head_b.at<float>(i-1)) ) ,
					cv::Point( bin_w*(i  ), hist_h - cvRound(user->_histogram._histogram_rgb_head_b.at<float>(i  )) ),
					cv::Scalar( 255, 0, 0), 2, 8, 0  );
		cv::line(	user->_histogram._histogram_rgb_vis_head, 
					cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram._histogram_rgb_head_g.at<float>(i-1)) ) ,
					cv::Point( bin_w*(i  ), hist_h - cvRound(user->_histogram._histogram_rgb_head_g.at<float>(i  )) ),
					cv::Scalar( 0, 255, 0), 2, 8, 0  );
		cv::line(	user->_histogram._histogram_rgb_vis_head, 
					cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram._histogram_rgb_head_r.at<float>(i-1)) ) ,
					cv::Point( bin_w*(i  ), hist_h - cvRound(user->_histogram._histogram_rgb_head_r.at<float>(i  )) ),
					cv::Scalar( 0, 0, 255), 2, 8, 0  );

		//TORSO
		cv::line(	user->_histogram._histogram_rgb_vis_torso, 
					cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram._histogram_rgb_torso_b.at<float>(i-1)) ) ,
					cv::Point( bin_w*(i  ), hist_h - cvRound(user->_histogram._histogram_rgb_torso_b.at<float>(i  )) ),
					cv::Scalar( 255, 0, 0), 2, 8, 0  );
		cv::line(	user->_histogram._histogram_rgb_vis_torso, 
					cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram._histogram_rgb_torso_g.at<float>(i-1)) ) ,
					cv::Point( bin_w*(i  ), hist_h - cvRound(user->_histogram._histogram_rgb_torso_g.at<float>(i  )) ),
					cv::Scalar( 0, 255, 0), 2, 8, 0  );
		cv::line(	user->_histogram._histogram_rgb_vis_torso, 
					cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram._histogram_rgb_torso_r.at<float>(i-1)) ) ,
					cv::Point( bin_w*(i  ), hist_h - cvRound(user->_histogram._histogram_rgb_torso_r.at<float>(i  )) ),
					cv::Scalar( 0, 0, 255), 2, 8, 0  );
		//LEGS
		cv::line(	user->_histogram._histogram_rgb_vis_legs, 
					cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram._histogram_rgb_legs_b.at<float>(i-1)) ) ,
					cv::Point( bin_w*(i  ), hist_h - cvRound(user->_histogram._histogram_rgb_legs_b.at<float>(i  )) ),
					cv::Scalar( 255, 0, 0), 2, 8, 0  );
		cv::line(	user->_histogram._histogram_rgb_vis_legs, 
					cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram._histogram_rgb_legs_g.at<float>(i-1)) ) ,
					cv::Point( bin_w*(i  ), hist_h - cvRound(user->_histogram._histogram_rgb_legs_g.at<float>(i  )) ),
					cv::Scalar( 0, 255, 0), 2, 8, 0  );
		cv::line(	user->_histogram._histogram_rgb_vis_legs, 
					cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram._histogram_rgb_legs_r.at<float>(i-1)) ) ,
					cv::Point( bin_w*(i  ), hist_h - cvRound(user->_histogram._histogram_rgb_legs_r.at<float>(i  )) ),
					cv::Scalar( 0, 0, 255), 2, 8, 0  );
	}

}

int thresh_rgb = 128;

float rgb_dist(int r1,int g1,int b1,int r2,int g2,int b2){
	return	sqrt((float)(r1-r2)*(r1-r2) + 
						(g1-g2)*(g1-g2) + 
						(b1-b2)*(b1-b2));
}

void rgb_histogram2(User* user){
	  
	user->_histogram._hist_img_result_rgb = cv::Mat( 480, 640 , CV_8UC3, cv::Scalar( 128,128,128) );

	openni::RGB888Pixel* ptr_clr = (openni::RGB888Pixel*)user->_kinect->get_color_frame_ref()->getData();
	uchar* mask_ptr = (uchar*)user->_mask_full.data;
	uchar* ptr_mask_head = (uchar*)user->_division3d._mask_head_orig.data;
	uchar* ptr_mask_torso = (uchar*)user->_division3d._mask_torso_orig.data;
	uchar* ptr_mask_legs = (uchar*)user->_division3d._mask_legs_orig.data;

	uint8_t* ptr_result = (uint8_t*)user->_histogram._hist_img_result_rgb.data;

	int idx = 0;
	int idx_clr = 0;
	int idx_hd = 0;
	user->_division3d._n_points = 0;

	int y_init = (BODY_PART_HD) ? user->_bbox.y/2 : user->_bbox.y;
	int y_end = (BODY_PART_HD) ? user->_bbox.y/2 + user->_bbox.height/2 : user->_bbox.y + user->_bbox.height;
	int x_init = (BODY_PART_HD) ? user->_bbox.x/2 : user->_bbox.x;
	int x_end = (BODY_PART_HD) ? user->_bbox.x/2 + user->_bbox.width/2 : user->_bbox.x + user->_bbox.width;

	for(int i = 0 ; i < 8 ; ++i)
		user->_histogram._hist_rgb[i] = 0;
	user->_histogram._counter = 0;

	float current_dist[8];

	for(int y = y_init ; y < y_end ; ++y){
		for(int x = x_init ; x < x_end ; ++x){
			idx = y * 640 + x;
			idx_clr = y * 640 * 3 + x* 3 + 0;
			int min_idx = 0;
			float min_dist = INT_MAX;
			if(mask_ptr[idx]){
				int r = ptr_clr[idx].r;
				int g = ptr_clr[idx].g;
				int b = ptr_clr[idx].b;

				current_dist[0] = rgb_dist(r,g,b, 0 , 0 , 0 ); /*0 0 0*/
				current_dist[1] = rgb_dist(r,g,b, 0 , 0 ,thresh_rgb); /*0 0 1*/
				current_dist[2] = rgb_dist(r,g,b, 0 ,thresh_rgb, 0 ); /*0 1 0*/
				current_dist[3] = rgb_dist(r,g,b, 0 ,thresh_rgb,thresh_rgb); /*0 1 1*/
				current_dist[4] = rgb_dist(r,g,b,thresh_rgb, 0 , 0 ); /*1 0 0*/
				current_dist[5] = rgb_dist(r,g,b,thresh_rgb, 0 ,thresh_rgb); /*1 0 1*/
				current_dist[6] = rgb_dist(r,g,b,thresh_rgb,thresh_rgb, 0 ); /*1 1 0*/
				current_dist[7] = rgb_dist(r,g,b,255,255,255); /*1 1 1*/

				for(int i = 0 ; i < 8 ; ++i){
					if(current_dist[i] < min_dist){
						min_dist = current_dist[i];
						min_idx = i;
					}
				}

				user->_histogram._hist_rgb[min_idx]++;

				if(min_idx == 0){ ptr_result[idx_clr+2] =  0 ; ptr_result[idx_clr+1] =  0 ; ptr_result[idx_clr+0] =  0 ;}
				if(min_idx == 1){ ptr_result[idx_clr+2] =  0 ; ptr_result[idx_clr+1] =  0 ; ptr_result[idx_clr+0] = 255;}
				if(min_idx == 2){ ptr_result[idx_clr+2] =  0 ; ptr_result[idx_clr+1] = 255; ptr_result[idx_clr+0] =  0 ;}
				if(min_idx == 3){ ptr_result[idx_clr+2] =  0 ; ptr_result[idx_clr+1] = 255; ptr_result[idx_clr+0] = 255;}
				if(min_idx == 4){ ptr_result[idx_clr+2] = 255; ptr_result[idx_clr+1] =  0 ; ptr_result[idx_clr+0] =  0 ;}
				if(min_idx == 5){ ptr_result[idx_clr+2] = 255; ptr_result[idx_clr+1] =  0 ; ptr_result[idx_clr+0] = 255;}
				if(min_idx == 6){ ptr_result[idx_clr+2] = 255; ptr_result[idx_clr+1] = 255; ptr_result[idx_clr+0] =  0 ;}
				if(min_idx == 7){ ptr_result[idx_clr+2] = 255; ptr_result[idx_clr+1] = 255; ptr_result[idx_clr+0] = 255;}
				//if( r < thresh_rgb   &&   g < thresh_rgb   &&   b < thresh_rgb) user->_histogram._hist_rgb[0]++; /*0 0 0*/
				//if( r < thresh_rgb   &&   g > thresh_rgb   &&   b < thresh_rgb) user->_histogram._hist_rgb[1]++; /*0 1 0*/
				//if( r < thresh_rgb   &&   g < thresh_rgb   &&   b > thresh_rgb) user->_histogram._hist_rgb[2]++; /*0 0 1*/
				//if( r < thresh_rgb   &&   g > thresh_rgb   &&   b > thresh_rgb) user->_histogram._hist_rgb[3]++; /*0 1 1*/
				//if( r > thresh_rgb   &&   g < thresh_rgb   &&   b < thresh_rgb) user->_histogram._hist_rgb[4]++; /*1 0 0*/
				//if( r > thresh_rgb   &&   g > thresh_rgb   &&   b < thresh_rgb) user->_histogram._hist_rgb[5]++; /*1 1 0*/
				//if( r > thresh_rgb   &&   g < thresh_rgb   &&   b > thresh_rgb) user->_histogram._hist_rgb[6]++; /*1 0 1*/
				//if( r > thresh_rgb   &&   g > thresh_rgb   &&   b > thresh_rgb) user->_histogram._hist_rgb[7]++; /*1 1 1*/

				user->_histogram._counter++;
			}
		}
	}

	for(int i = 0 ; i < 8 ; ++i)
		user->_histogram._hist_rgb[i] /= (float)user->_histogram._counter;

	int hist_w = 256; int hist_h = 200;

	user->_histogram._hist_vis_rgb = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 128,128,128) );

	int bar_width = hist_w/8;
	for( int i = 0; i < 8; i++ ){
		int height = hist_h - user->_histogram._hist_rgb[i] * hist_h;

		cv::Scalar color;
		if(i == 0){ color.val[2] =  0 ; color.val[1] =  0 ; color.val[0] =  0 ;} /*0 0 0*/
		if(i == 1){ color.val[2] =  0 ; color.val[1] =  0 ; color.val[0] = 255;} /*0 0 1*/
		if(i == 2){ color.val[2] =  0 ; color.val[1] = 255; color.val[0] =  0 ;} /*0 1 0*/
		if(i == 3){ color.val[2] =  0 ; color.val[1] = 255; color.val[0] = 255;} /*0 1 1*/
		if(i == 4){ color.val[2] = 255; color.val[1] =  0 ; color.val[0] =  0 ;} /*1 0 0*/
		if(i == 5){ color.val[2] = 255; color.val[1] =  0 ; color.val[0] = 255;} /*1 0 1*/
		if(i == 6){ color.val[2] = 255; color.val[1] = 255; color.val[0] =  0 ;} /*1 1 0*/
		if(i == 7){ color.val[2] = 255; color.val[1] = 255; color.val[0] = 255;} /*1 1 1*/


		cv::rectangle(user->_histogram._hist_vis_rgb,cv::Rect(i*bar_width,height,bar_width,hist_h-height),color,-1);
	}


}

void hsv_histogram(User* user){
	cv::cvtColor( user->_color_full, user->_histogram._hsv_user, CV_BGR2HSV );

	cv::vector<cv::Mat> hsv_planes;
	cv::split( user->_histogram._hsv_user, hsv_planes );

	/// Using 30 bins for hue and 32 for saturation
	int h_bins = 10; int s_bins = 12;
	int histSize[] = { h_bins, s_bins };

	float range1[] = { 0, 256 };
	float range2[] = { 0, 180 };
	// hue varies from 0 to 256, saturation from 0 to 180
	const float *h_ranges = {range1};
	const float *s_ranges = {range2};

	const float* ranges[] = { h_ranges, s_ranges };

	// Use the o-th and 1-st channels
	int channels[] = { 0, 1 };

	/// Histograms
	bool uniform = true; bool accumulate = false;
	//cv::MatND hist_base;

	int hist_w = 256; int hist_h = 200;
	int bin_w = cvRound( (double) hist_w/h_bins );

	user->_histogram._hsv_hist_h = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
	user->_histogram._hsv_hist_s = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

	user->_histogram._histogram_hsv_vis_full = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
	user->_histogram._histogram_hsv_vis_head = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
	user->_histogram._histogram_hsv_vis_torso = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
	user->_histogram._histogram_hsv_vis_legs = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );



	cv::calcHist( &hsv_planes[0], 1, 0, user->_mask_full, user->_histogram._histogram_hsv_full_h, 1, &h_bins, &h_ranges, uniform, accumulate );
	cv::calcHist( &hsv_planes[1], 1, 0, user->_mask_full, user->_histogram._histogram_hsv_full_s, 1, &h_bins, &s_ranges, uniform, accumulate );
	cv::normalize(user->_histogram._histogram_hsv_full_h, user->_histogram._histogram_hsv_full_h, 0, user->_histogram._hsv_hist_h.rows, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(user->_histogram._histogram_hsv_full_s, user->_histogram._histogram_hsv_full_s, 0, user->_histogram._hsv_hist_h.rows, cv::NORM_MINMAX, -1, cv::Mat() );

	cv::calcHist( &hsv_planes[0], 1, 0, user->_division3d._mask_head, user->_histogram._histogram_hsv_head_h, 1, &h_bins, &h_ranges, uniform, accumulate );
	cv::calcHist( &hsv_planes[1], 1, 0, user->_division3d._mask_head, user->_histogram._histogram_hsv_head_s, 1, &h_bins, &s_ranges, uniform, accumulate );
	cv::normalize(user->_histogram._histogram_hsv_head_h, user->_histogram._histogram_hsv_head_h, 0, user->_histogram._hsv_hist_h.rows, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(user->_histogram._histogram_hsv_head_s, user->_histogram._histogram_hsv_head_s, 0, user->_histogram._hsv_hist_h.rows, cv::NORM_MINMAX, -1, cv::Mat() );

	cv::calcHist( &hsv_planes[0], 1, 0, user->_division3d._mask_torso, user->_histogram._histogram_hsv_torso_h, 1, &h_bins, &h_ranges, uniform, accumulate );
	cv::calcHist( &hsv_planes[1], 1, 0, user->_division3d._mask_torso, user->_histogram._histogram_hsv_torso_s, 1, &h_bins, &s_ranges, uniform, accumulate );
	cv::normalize(user->_histogram._histogram_hsv_torso_h, user->_histogram._histogram_hsv_torso_h, 0, user->_histogram._hsv_hist_h.rows, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(user->_histogram._histogram_hsv_torso_s, user->_histogram._histogram_hsv_torso_s, 0, user->_histogram._hsv_hist_h.rows, cv::NORM_MINMAX, -1, cv::Mat() );

	cv::calcHist( &hsv_planes[0], 1, 0, user->_division3d._mask_legs, user->_histogram._histogram_hsv_legs_h, 1, &h_bins, &h_ranges, uniform, accumulate );
	cv::calcHist( &hsv_planes[1], 1, 0, user->_division3d._mask_legs, user->_histogram._histogram_hsv_legs_s, 1, &h_bins, &s_ranges, uniform, accumulate );
	cv::normalize(user->_histogram._histogram_hsv_legs_h, user->_histogram._histogram_hsv_legs_h, 0, user->_histogram._hsv_hist_h.rows, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(user->_histogram._histogram_hsv_legs_s, user->_histogram._histogram_hsv_legs_s, 0, user->_histogram._hsv_hist_h.rows, cv::NORM_MINMAX, -1, cv::Mat() );
	
	/// Calculate the histograms for the HSV images
	//Full User
	//cv::calcHist( &user->_histogram._hsv_user, 1, channels, user->_mask_full, user->_histogram._histogram_hsv_full, 2, histSize, ranges, true, false );
	//cv::normalize( user->_histogram._histogram_hsv_full, user->_histogram._histogram_hsv_full, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
	////HEAD
	//cv::calcHist( &user->_histogram._hsv_user, 1, channels, user->_division3d._mask_head, user->_histogram._histogram_hsv_head, 2, histSize, ranges, true, false );
	//cv::normalize( user->_histogram._histogram_hsv_head, user->_histogram._histogram_hsv_head, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
	////TORSO
	//cv::calcHist( &user->_histogram._hsv_user, 1, channels, user->_division3d._mask_torso, user->_histogram._histogram_hsv_torso, 2, histSize, ranges, true, false );
	//cv::normalize( user->_histogram._histogram_hsv_torso, user->_histogram._histogram_hsv_torso, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
	////LEGS
	//cv::calcHist( &user->_histogram._hsv_user, 1, channels, user->_division3d._mask_legs, user->_histogram._histogram_hsv_legs, 2, histSize, ranges, true, false );
	//cv::normalize( user->_histogram._histogram_hsv_legs, user->_histogram._histogram_hsv_legs, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );


	//cv::applyColorMap(hsv_planes[0],user->_histogram._pseudo_h,9);
	//cv::applyColorMap(hsv_planes[1],user->_histogram._pseudo_s,9);

	//cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

	// Draw for each channel
	for( int i = 1; i < h_bins; i++ )	{
		//FULL
		cv::line( user->_histogram._histogram_hsv_vis_full, cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram._histogram_hsv_full_h.at<float>(i-1)) ) ,
						cv::Point( bin_w*(i), hist_h - cvRound(user->_histogram._histogram_hsv_full_h.at<float>(i)) ),
						cv::Scalar( 255, 0, 0), 2, 8, 0  );
		cv::line( user->_histogram._histogram_hsv_vis_full, cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram._histogram_hsv_full_s.at<float>(i-1)) ) ,
						cv::Point( bin_w*(i), hist_h - cvRound(user->_histogram._histogram_hsv_full_s.at<float>(i)) ),
						cv::Scalar( 0, 255, 0), 2, 8, 0  );

		//HEAD
		cv::line( user->_histogram._histogram_hsv_vis_head, cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram._histogram_hsv_head_h.at<float>(i-1)) ) ,
						cv::Point( bin_w*(i), hist_h - cvRound(user->_histogram._histogram_hsv_head_h.at<float>(i)) ),
						cv::Scalar( 255, 0, 0), 2, 8, 0  );
		cv::line( user->_histogram._histogram_hsv_vis_head, cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram._histogram_hsv_head_s.at<float>(i-1)) ) ,
						cv::Point( bin_w*(i), hist_h - cvRound(user->_histogram._histogram_hsv_head_s.at<float>(i)) ),
						cv::Scalar( 0, 255, 0), 2, 8, 0  );

		//TORSO
		cv::line( user->_histogram._histogram_hsv_vis_torso, cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram._histogram_hsv_torso_h.at<float>(i-1)) ) ,
						cv::Point( bin_w*(i), hist_h - cvRound(user->_histogram._histogram_hsv_torso_h.at<float>(i)) ),
						cv::Scalar( 255, 0, 0), 2, 8, 0  );
		cv::line( user->_histogram._histogram_hsv_vis_torso, cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram._histogram_hsv_torso_s.at<float>(i-1)) ) ,
						cv::Point( bin_w*(i), hist_h - cvRound(user->_histogram._histogram_hsv_torso_s.at<float>(i)) ),
						cv::Scalar( 0, 255, 0), 2, 8, 0  );

		//LEGS
		cv::line( user->_histogram._histogram_hsv_vis_legs, cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram._histogram_hsv_legs_h.at<float>(i-1)) ) ,
						cv::Point( bin_w*(i), hist_h - cvRound(user->_histogram._histogram_hsv_legs_h.at<float>(i)) ),
						cv::Scalar( 255, 0, 0), 2, 8, 0  );
		cv::line( user->_histogram._histogram_hsv_vis_legs, cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram._histogram_hsv_legs_s.at<float>(i-1)) ) ,
						cv::Point( bin_w*(i), hist_h - cvRound(user->_histogram._histogram_hsv_legs_s.at<float>(i)) ),
						cv::Scalar( 0, 255, 0), 2, 8, 0  );
	}
}

int thresh_hsv_black_v = 60;
int thresh_hsv_white_s = 40;
int thresh_hsv_white_v = 150;

void hsv_histogram2(User* user){
	cv::cvtColor( user->_color_full, user->_histogram._hsv_user, CV_BGR2HSV );

	uchar* mask_ptr = (uchar*)user->_mask_full.data;
	uchar* ptr_mask_head = (uchar*)user->_division3d._mask_head_orig.data;
	uchar* ptr_mask_torso = (uchar*)user->_division3d._mask_torso_orig.data;
	uchar* ptr_mask_legs = (uchar*)user->_division3d._mask_legs_orig.data;

	uint8_t* ptr_clr = (uint8_t*)user->_histogram._hsv_user.data;
	
	user->_histogram._hist_img_result_hsv = cv::Mat( 480, 640 , CV_8UC3, cv::Scalar( 128,128,128) );
	uint8_t* ptr_result = (uint8_t*)user->_histogram._hist_img_result_hsv.data;

	int idx = 0;
	int idx_clr = 0;
	int idx_hd = 0;
	user->_division3d._n_points = 0;

	int y_init = (BODY_PART_HD) ? user->_bbox.y/2 : user->_bbox.y;
	int y_end  = (BODY_PART_HD) ? user->_bbox.y/2 + user->_bbox.height/2 : user->_bbox.y + user->_bbox.height;
	int x_init = (BODY_PART_HD) ? user->_bbox.x/2 : user->_bbox.x;
	int x_end  = (BODY_PART_HD) ? user->_bbox.x/2 + user->_bbox.width/2 : user->_bbox.x + user->_bbox.width;

	for(int i = 0 ; i < 9 ; ++i)
		user->_histogram._hist_hsv[i] = 0;
	user->_histogram._counter = 0;

	float current_dist[9];

	for(int y = y_init ; y < y_end ; ++y){
		for(int x = x_init ; x < x_end ; ++x){
			idx = y * 640 + x;
			idx_clr = y * 640 * 3 + x* 3;
			int min_idx = 0;
			float min_dist = INT_MAX;
			if(ptr_mask_torso[idx]){//if(mask_ptr[idx]){
				int h = ptr_clr[idx_clr + 0];
				int s = ptr_clr[idx_clr + 1];
				int v = ptr_clr[idx_clr + 2];

				//int r = ptr_clr_rgb[idx_clr + 2];
				//int g = ptr_clr_rgb[idx_clr + 1];
				//int b = ptr_clr_rgb[idx_clr + 0];

				if( v <  thresh_hsv_black_v){
					ptr_result[idx_clr+2] =  0 ; ptr_result[idx_clr+1] =  0 ; ptr_result[idx_clr+0] =  0 ;
					current_dist[0] = 0;
				}
				else
				if( v > thresh_hsv_white_v  &&  s < thresh_hsv_white_s) {
					ptr_result[idx_clr+2] =  0 ; ptr_result[idx_clr+1] =  0 ; ptr_result[idx_clr+0] = 255;
					current_dist[7] = 0;
				}
				//else
				//if( h <  20 && s < 50 && v > 200){
				//	current_dist[8] = 0;
				//}
				else{
					current_dist[0] = INT_MAX ;
					current_dist[1] = (h - 120) * (h - 120);	/*0 0 1*/
					current_dist[2] = (h -  60) * (h -  60);	/*0 1 0*/
					current_dist[3] = (h -  90) * (h -  90);	/*0 1 1*/
					current_dist[4] = (h -  0 ) * (h -  0 );	/*1 0 0*/
					current_dist[5] = (h - 150) * (h - 150);	/*1 0 1*/
					current_dist[6] = (h -  20) * (h -  20);	/*1 1 0*/
					current_dist[7] = INT_MAX ;
					current_dist[8] = INT_MAX ;
				}

				for(int i = 0 ; i < 9 ; ++i){
					if(current_dist[i] < min_dist){
						min_dist = current_dist[i];
						min_idx = i;
					}
				}

				if(min_idx == 0){ ptr_result[idx_clr+0] =  0 ; ptr_result[idx_clr+1] =  0 ; ptr_result[idx_clr+2] =  0 ;} /*0 0 0*/
				if(min_idx == 1){ ptr_result[idx_clr+0] = 120; ptr_result[idx_clr+1] = 255; ptr_result[idx_clr+2] = 255;} /*0 0 1*/
				if(min_idx == 2){ ptr_result[idx_clr+0] =  60; ptr_result[idx_clr+1] = 255; ptr_result[idx_clr+2] = 255;} /*0 1 0*/
				if(min_idx == 3){ ptr_result[idx_clr+0] =  90; ptr_result[idx_clr+1] = 255; ptr_result[idx_clr+2] = 255;} /*0 1 1*/
				if(min_idx == 4){ ptr_result[idx_clr+0] =  0 ; ptr_result[idx_clr+1] = 255; ptr_result[idx_clr+2] = 255;} /*1 0 0*/
				if(min_idx == 5){ ptr_result[idx_clr+0] = 0; ptr_result[idx_clr+1] = 255; ptr_result[idx_clr+2] = 255;}   /*1 0 1*/
				if(min_idx == 6){ ptr_result[idx_clr+0] =  30; ptr_result[idx_clr+1] = 255; ptr_result[idx_clr+2] = 255;} /*1 1 0*/
				if(min_idx == 7){ ptr_result[idx_clr+0] =  0 ; ptr_result[idx_clr+1] =  0 ; ptr_result[idx_clr+2] = 255;} /*1 1 1*/
				if(min_idx == 8){ ptr_result[idx_clr+0] =  0 ; ptr_result[idx_clr+1] =  0 ; ptr_result[idx_clr+2] = 190;} /*1 1 1*/

				if(min_idx == 4)
					user->_histogram._hist_hsv[min_idx+1]++;
				else
					user->_histogram._hist_hsv[min_idx]++;

				user->_histogram._counter++;
			}
		}
	}

	for(int i = 0 ; i < 9 ; ++i)
		user->_histogram._hist_hsv[i] /= (float)user->_histogram._counter;

	int hist_w = 256; int hist_h = 200;

	user->_histogram._hist_vis_hsv = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 128,128,128) );

	int bar_width = hist_w/9;
	if(user->_histogram._counter){
		for( int i = 0; i < 9; i++ ){
			int height = hist_h - user->_histogram._hist_hsv[i] * hist_h;

			cv::Scalar color;
			if(i == 0){ color.val[0] =  0 ; color.val[1] =  0 ; color.val[2] =  0 ;} /*0 0 0*/
			if(i == 1){ color.val[0] = 120; color.val[1] = 255; color.val[2] = 255;} /*0 0 1*/
			if(i == 2){ color.val[0] =  60; color.val[1] = 255; color.val[2] = 255;} /*0 1 0*/
			if(i == 3){ color.val[0] =  90; color.val[1] = 255; color.val[2] = 255;} /*0 1 1*/
			if(i == 4){ color.val[0] =  0 ; color.val[1] = 255; color.val[2] = 255;} /*1 0 0*/
			if(i == 5){ color.val[0] = 0; color.val[1] = 255; color.val[2] = 255;} /*1 0 1*/
			if(i == 6){ color.val[0] =  30; color.val[1] = 255; color.val[2] = 255;} /*1 1 0*/
			if(i == 7){ color.val[0] =  0 ; color.val[1] =  0 ; color.val[2] = 255;} /*1 1 1*/
			if(i == 8){ color.val[0] =  0 ; color.val[1] =  0 ; color.val[2] = 190;} /*1 1 1*/


			cv::rectangle(user->_histogram._hist_vis_hsv,cv::Rect(i*bar_width,height,bar_width,hist_h-height),color,-1);
		}
	}
	cv::cvtColor(user->_histogram._hist_img_result_hsv,user->_histogram._hist_img_result_hsv_cvt,CV_HSV2BGR);
	cv::cvtColor(user->_histogram._hist_vis_hsv,user->_histogram._hist_vis_hsv_cvt,CV_HSV2BGR);
}

int non_zeros_val = 4;
int const max_non_zeros = 64;

cv::Mat get_hogdescriptor_visu(User* user, cv::Mat& origImg, cv::vector<float>& descriptorValues){   
    cv::Mat color_origImg;
    cvtColor(origImg, color_origImg, CV_GRAY2RGB);
 
    float zoomFac = 3;
    cv::Mat visu;
    resize(color_origImg, visu, cv::Size(color_origImg.cols*zoomFac, color_origImg.rows*zoomFac));
 
    int blockSize       = 16;
    int cellSize        = 8;
    int gradientBinSize = 9;
    float radRangeForOneBin = M_PI/(float)gradientBinSize; // dividing 180° into 9 bins, how large (in rad) is one bin?
 
    // prepare data structure: 9 orientation / gradient strenghts for each cell
    int cells_in_x_dir = 64 / cellSize;
    int cells_in_y_dir = 128 / cellSize;
    int totalnrofcells = cells_in_x_dir * cells_in_y_dir;
    float*** gradientStrengths = new float**[cells_in_y_dir];
    int** cellUpdateCounter   = new int*[cells_in_y_dir];
    for (int y=0; y<cells_in_y_dir; y++)
    {
        gradientStrengths[y] = new float*[cells_in_x_dir];
        cellUpdateCounter[y] = new int[cells_in_x_dir];
        for (int x=0; x<cells_in_x_dir; x++)
        {
            gradientStrengths[y][x] = new float[gradientBinSize+1];
            cellUpdateCounter[y][x] = 0;
 
            for (int bin=0; bin<gradientBinSize+1; bin++)
                gradientStrengths[y][x][bin] = 0.0;
        }
    }
 
    // nr of blocks = nr of cells - 1
    // since there is a new block on each cell (overlapping blocks!) but the last one
    int blocks_in_x_dir = cells_in_x_dir - 1;
    int blocks_in_y_dir = cells_in_y_dir - 1;
 
    // compute gradient strengths per cell
    int descriptorDataIdx = 0;
    int cellx = 0;
    int celly = 0;

	int aux_y = (user->_features._rect_torso_roi.y*2) / 16 + 1;
	int aux_x = (user->_features._rect_torso_roi.x*2) / 16 + 1;
 
    for (int blockx=0; blockx<blocks_in_x_dir; blockx++)
    {
        for (int blocky=0; blocky<blocks_in_y_dir; blocky++)            
        {
            // 4 cells per block ...
            for (int cellNr=0; cellNr<4; cellNr++)
            {
                // compute corresponding cell nr
                int cellx = blockx;
                int celly = blocky;
                if (cellNr==1) celly++;
                if (cellNr==2) cellx++;
                if (cellNr==3){
                    cellx++;
                    celly++;
                }
 
				cv::Rect rect(cellx*8,celly*8,8,8);
				int non_zeros = cv::countNonZero(user->_features._canny_torso(rect));

				if(non_zeros > non_zeros_val ){
					for (int bin=0; bin<gradientBinSize; bin++)
					{
						float gradientStrength = descriptorValues[ descriptorDataIdx ];
						descriptorDataIdx++;
 
						gradientStrengths[celly][cellx][bin] += gradientStrength;
 
					} // for (all bins)
 
					
				}
				else{
					gradientStrengths[celly][cellx][9] += 0.10;//((float)((64-non_zeros_val) - non_zeros)) / (float)max_non_zeros;
					cv::circle(visu,cv::Point((cellx*8+4)*zoomFac,(celly*8+4)*zoomFac),5,cv::Scalar(0,0,255),-1);
				}
				// note: overlapping blocks lead to multiple updates of this sum!
				// we therefore keep track how often a cell was updated,
				// to compute average gradient strengths
				cellUpdateCounter[celly][cellx]++;
            } // for (all cells)
        } // for (all block x pos)
    } // for (all block y pos)
 
    // compute average gradient strengths
    for (int celly=0; celly<cells_in_y_dir; celly++){
        for (int cellx=0; cellx<cells_in_x_dir; cellx++){
 			float NrUpdatesForThisCell = (float)cellUpdateCounter[celly][cellx];
             // compute average gradient strenghts for each gradient bin direction
            for (int bin=0; bin<gradientBinSize; bin++){
                gradientStrengths[celly][cellx][bin] /= NrUpdatesForThisCell;
            }
        }
    }
 
 
    //cout << "descriptorDataIdx = " << descriptorDataIdx << endl;
	for(int j = 0 ; j < 10 ; ++j){
		user->_features._hog_hist_torso[j] = 0;
	}
	int counter = 0;


	
	//user->_features._canny_torso;
	//if(aux_y > 1) aux_y+=1;
	//if(aux_x > 1) aux_x+=1;

    // draw cells
    for (int celly=aux_y; celly<cells_in_y_dir - aux_y; celly++)
    {
        for (int cellx=aux_x; cellx<cells_in_x_dir - aux_x; cellx++)
        {
            int drawX = cellx * cellSize;
            int drawY = celly * cellSize;
 
            int mx = drawX + cellSize/2;
            int my = drawY + cellSize/2;
 
            rectangle(visu, cv::Point(drawX*zoomFac,drawY*zoomFac), cv::Point((drawX+cellSize)*zoomFac,(drawY+cellSize)*zoomFac), CV_RGB(100,100,100), 1);
 
			counter++;
            // draw in each cell all 9 gradient strengths
            for (int bin=0; bin<gradientBinSize+1; bin++)
            {
                float currentGradStrength = gradientStrengths[celly][cellx][bin];
 
				user->_features._hog_hist_torso[bin] += currentGradStrength;

                // no line to draw?
                if (currentGradStrength==0)
                    continue;
 
                float currRad = bin * radRangeForOneBin + radRangeForOneBin/2;
 
                float dirVecX = cos( currRad );
                float dirVecY = sin( currRad );
                float maxVecLen = cellSize/2;
                float scale = 2.5; // just a visualization scale, to see the lines better
 
                // compute line coordinates
                float x1 = mx - dirVecX * currentGradStrength * maxVecLen * scale;
                float y1 = my - dirVecY * currentGradStrength * maxVecLen * scale;
                float x2 = mx + dirVecX * currentGradStrength * maxVecLen * scale;
                float y2 = my + dirVecY * currentGradStrength * maxVecLen * scale;
 
                // draw gradient visualization
                line(visu, cv::Point(x1*zoomFac,y1*zoomFac), cv::Point(x2*zoomFac,y2*zoomFac), CV_RGB(0,255,0), 1);
 
            } // for (all bins)
        } // for (cellx)
    } // for (celly)
 
	for(int j = 0 ; j < 10 ; ++j){
		user->_features._hog_hist_torso[j] /= counter;
	}

    // don't forget to free memory allocated by helper data structures!
    for (int y=0; y<cells_in_y_dir; y++)
    {
      for (int x=0; x<cells_in_x_dir; x++)
      {
           delete[] gradientStrengths[y][x];            
      }
      delete[] gradientStrengths[y];
      delete[] cellUpdateCounter[y];
    }
    delete[] gradientStrengths;
    delete[] cellUpdateCounter;
 
    return visu;
 
} // get_hogdescriptor_visu

double fitt_image(cv::Mat& orig, cv::Mat& out, int out_width = 640, int out_height = 480, cv::Rect* roi_out = 0){
		//Test Input
		if(orig.cols <= 0 || orig.rows <= 0) return -1;

		//Create Output
		out = cv::Mat::zeros(cv::Size(out_width,out_height),orig.type());

		//Calc ROI and Resize Image
		if(!roi_out) roi_out = new cv::Rect();
		cv::Mat temp;
		int w,h;
		double output_ratio = (double)out.cols / (double)out.rows;
		double input_ratio = (double)orig.cols / (double)orig.rows;

		double ratio = 1.0;

		if(input_ratio < output_ratio){
			ratio = (float)out.rows/(float)orig.rows;
			w = orig.cols *ratio;
			h = orig.rows *ratio;

			cv::resize(orig,temp,cv::Size(w,h),0.0,0.0,cv::INTER_CUBIC);
		
			roi_out->x = (out.cols - w) / 2.0;
			roi_out->y = 0;
			roi_out->width = w;
			roi_out->height = h;
		}
		else{
			ratio = (float)out.cols/(float)orig.cols;
			w = orig.cols *ratio;
			h = orig.rows *ratio;

			cv::resize(orig,temp,cv::Size(w,h),0.0,0.0,cv::INTER_CUBIC);
		
			roi_out->x = 0;
			roi_out->y = (out.rows - h) / 2.0;
			roi_out->width = w;
			roi_out->height = h;
		}

		//Copy to Destination
		temp.copyTo(out(*roi_out));

		return ratio;
	}

bool calc_hog(User* user){
	if(!user) return false;
	 
	cv::Mat gray_bbox;
	cv::Mat gray_mask;
	
	cv::cvtColor(user->_color_full,user->_features._gray_full,CV_RGB2GRAY);
	user->_features._gray_full(user->_bbox).copyTo(gray_bbox);
	gray_bbox.copyTo(gray_mask,user->_mask_user);

	fitt_image(gray_bbox,user->_features._gray_user,64,128);
	fitt_image(gray_mask,user->_features._gray_user_mask,64,128);

	//cv::Size s_bbox((gray_bbox.cols % 2) ? gray_bbox.cols-1 : gray_bbox.cols,
	//				(gray_bbox.rows % 2) ? gray_bbox.rows-1 : gray_bbox.rows);
	//cv::Size s_mask((gray_mask.cols % 2) ? gray_mask.cols-1 : gray_mask.cols,
	//				(gray_mask.rows % 2) ? gray_mask.rows-1 : gray_mask.rows);
	//cv::Size s_bbox(gray_bbox.cols,gray_bbox.rows);
	//cv::Size s_mask(gray_mask.cols,gray_mask.rows);

	//while(s_bbox.width % 8)
	//	s_bbox.width--;
	//while(s_bbox.height % 8)
	//	s_bbox.height--;
	//while(s_mask.width % 8)
	//	s_mask.width--;
	//while(s_mask.height % 8)
	//	s_mask.height--;


	//cv::resize(gray_bbox,user->_gray_user_bbox,s_bbox);
	//cv::resize(gray_mask,user->_gray_user_mask,s_mask);
	
	//cv::Size cellSize(user->_gray_user_bbox.cols / 16,user->_gray_user_bbox.rows / 16);
	//cv::Size winSize(user->_gray_user_bbox.cols,user->_gray_user_bbox.rows);		//(128,64)
	//cv::Size blockSize(user->_gray_user_bbox.cols / 8,user->_gray_user_bbox.rows / 8);
	//cv::Size blockStride((winSize.width - blockSize.width)/16,(winSize.height - blockSize.height)/16);
	//

	//int q00 = blockSize.width % cellSize.width;
	//int q01 = blockSize.height % cellSize.height;

	//int q1 = winSize.width - blockSize.width;
	//int q2 = q1 % blockStride.width;
	//int q3 = winSize.height - blockSize.height;
	//int q4 = q3 % blockStride.height;

	cv::HOGDescriptor		hog_bbox;//(//;//(winSize, blockSize, blockStride, cellSize, 9);
									//cv::Size(128,64), //winSize cv::Size(128,64)
									//cv::Size(16,16), //blocksize cv::Size(16,16)
									//cv::Size(8,8), //blockStride, cv::Size(8,8)
									//cv::Size(8,8), //cellSize, cv::Size(8,8)
									//9); //nbins,
	cv::vector<float>		ders_bbox;
	cv::vector<cv::Point>	locs_bbox;

	hog_bbox.compute(user->_features._gray_user,ders_bbox,cv::Size(32,32),cv::Size(0,0),locs_bbox);

	if(ders_bbox.size())
		user->_features._hog_vis_bbox = get_hogdescriptor_visu(user, user->_features._gray_user,ders_bbox);
	

	cv::HOGDescriptor		hog_mask;//(//;//(winSize, blockSize, blockStride, cellSize, 9);
									//cv::Size(128,64), //winSize cv::Size(128,64)
									//cv::Size(16,16), //blocksize cv::Size(16,16)
									//cv::Size(8,8), //blockStride, cv::Size(8,8)
									//cv::Size(8,8), //cellSize, cv::Size(8,8)
									//9); //nbins,
	cv::vector<float>		ders_mask;
	cv::vector<cv::Point>	locs_mask;

	hog_mask.compute(user->_features._gray_user_mask,ders_mask,cv::Size(32,32),cv::Size(0,0),locs_mask);
		
	user->_features._hog_vis_mask= get_hogdescriptor_visu(user, user->_features._gray_user_mask,ders_mask);
	

	//Calc Histograms
	for(int i = 0 ; i < ders_mask.size() ; i+=9){
		for(int j = 0 ; j < 9 ; ++j){
			user->_features._hog_hist_bbox[j] = 0;
			user->_features._hog_hist_mask[j] = 0;
		}
	}
	for(int i = 0 ; i < ders_mask.size() ; i+=9){
		for(int j = 0 ; j < 9; ++j){
			user->_features._hog_hist_bbox[j] += ders_bbox[i + j];
			user->_features._hog_hist_mask[j] += ders_mask[i + j];
		}
	}
	
	int n_bins = 9;
	int hist_w = 270; int hist_h = 200;
	int bin_w = cvRound( (double) hist_w/n_bins );
	

	user->_features._hog_vis_hist_bbox = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
	user->_features._hog_vis_hist_mask = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

	cv::normalize(user->_features._hog_hist_bbox,user->_features._hog_hist_bbox);
	cv::normalize(user->_features._hog_hist_mask,user->_features._hog_hist_mask);

	for( int i = 1; i < n_bins; i++ )	{
		cv::line( user->_features._hog_vis_hist_bbox, 
						cv::Point( bin_w*(i-1), hist_h - hist_h*user->_features._hog_hist_bbox[i-1]),
						cv::Point( bin_w*(i)  , hist_h - hist_h*user->_features._hog_hist_bbox[i]  ),
						cv::Scalar( 255, 0, 0), 2, 8, 0  );

		cv::line( user->_features._hog_vis_hist_mask, 
						cv::Point( bin_w*(i-1), hist_h - hist_h*user->_features._hog_hist_mask[i-1]),
						cv::Point( bin_w*(i)  , hist_h - hist_h*user->_features._hog_hist_mask[i]  ),
						cv::Scalar( 0, 255, 0), 2, 8, 0  );
	}

	return user->_features._hog_vis_mask.rows && user->_features._hog_vis_bbox.rows;
}

void calc_center_rect(User* user){
	uchar* mask_ptr = (uchar*)user->_division3d._mask_torso_orig.data;

	int idx = 0;
	int idx2 = 0;
	int y_init = (BODY_PART_HD) ? user->_bbox.y/2 : user->_bbox.y;
	int y_end = (BODY_PART_HD) ? user->_bbox.y/2 + user->_bbox.height/2 : user->_bbox.y + user->_bbox.height;
	int x_init = (BODY_PART_HD) ? user->_bbox.x/2 : user->_bbox.x;
	int x_end = (BODY_PART_HD) ? user->_bbox.x/2 + user->_bbox.width/2 : user->_bbox.x + user->_bbox.width;

	cv::Point center(user->_centroid_2d.x, user->_centroid_2d.y);

	int up_y = y_init;
	int bottom_y = 0;
	int right_x = 0;
	int left_x = 640;

	//DOWN
	for(int y = center.y ; y < y_end ; ++y){
		idx = y * 640 + center.x;
		if(!mask_ptr[idx]){
			bottom_y = y;
			break;
		}
	}
	//UP
	for(int y = center.y-1 ; y > y_init ; --y){
		idx =y * 640 + center.x;
		if(!mask_ptr[idx]){
			up_y = y;
			break;
		}
	}
	
	int diff_y = (bottom_y - up_y) * 0.1;
	bottom_y -= diff_y;
	up_y += diff_y;
	
	//RIGHT
	for(int x = center.x ; x < x_end ; ++x){
		idx = bottom_y * 640 + x;
		idx2 = up_y * 640 + x;
		if(!mask_ptr[idx] || !mask_ptr[idx2]){
			if(x > right_x){
				right_x = x;
				break;
			}
		}
	}
	//LEFT
	for(int x = center.x-1 ; x > x_init ; --x){
		idx = bottom_y * 640 + x;
		idx2 = up_y * 640 + x;
		if(!mask_ptr[idx] || !mask_ptr[idx2]){
			if(x < left_x){
				left_x = x;
				break;
			}
		}
	}

	int diff_x = (right_x - left_x) * 0.2;
	right_x -= diff_x;
	left_x += diff_x;

	user->_features._rect_torso.x = left_x;
	user->_features._rect_torso.width = right_x-left_x;
	user->_features._rect_torso.y = up_y;
	user->_features._rect_torso.height = bottom_y-up_y;

	cv::Mat img;
	user->_color_full.copyTo(img,user->_mask_orig);

	cv::circle(img,cv::Point(user->_features._rect_torso.x , user->_features._rect_torso.y),5,cv::Scalar(0,255,255),-1);
	cv::circle(img,cv::Point(user->_features._rect_torso.x , user->_features._rect_torso.y + user->_features._rect_torso.height),5,cv::Scalar(0,255,255),-1);
	cv::circle(img,cv::Point(user->_features._rect_torso.x + user->_features._rect_torso.width, user->_features._rect_torso.y),5,cv::Scalar(0,255,255),-1);
	cv::circle(img,cv::Point(user->_features._rect_torso.x + user->_features._rect_torso.width, user->_features._rect_torso.y + user->_features._rect_torso.height),5,cv::Scalar(0,255,255),-1);


	//cv::imshow("points",img);
}

int lowThreshold = 50;
int const max_lowThreshold = 100;

bool calc_center_canny(User* user){
	if(!user || user->_features._rect_torso.width < 10 || user->_features._rect_torso.width < 10 || user->_features._rect_torso.height < 10) return false;
	 
	cv::Mat gray_bbox;
	cv::Mat gray_mask;
	cv::Mat gray_torso;

	cv::cvtColor(user->_color_full,user->_features._gray_full,CV_RGB2GRAY);
	user->_features._gray_full(user->_bbox).copyTo(gray_bbox);
	gray_bbox.copyTo(gray_mask,user->_mask_user);
	user->_features._gray_full(user->_features._rect_torso).copyTo(gray_torso);

	fitt_image(gray_bbox,user->_features._gray_user,64,128);
	fitt_image(gray_mask,user->_features._gray_user_mask,64,128);
	fitt_image(gray_torso,user->_features._gray_torso,64,128, &user->_features._rect_torso_roi);

	int edgeThresh = 1;
	
	int ratio = 3;
	int kernel_size = 3;

	user->_features._gray_torso.copyTo(user->_features._canny_torso);
	//cv::cvtColor(user->_features._gray_torso,gray,CV_BGR2GRAY);

	cv::Canny( user->_features._canny_torso, user->_features._canny_torso, lowThreshold, lowThreshold*ratio, kernel_size );

	return true;
}

bool calc_center_hog(User* user){
	if(!user) return false;

	cv::HOGDescriptor		hog_torso;//(//;//(winSize, blockSize, blockStride, cellSize, 9);
									//cv::Size(128,64), //winSize cv::Size(128,64)
									//cv::Size(16,16), //blocksize cv::Size(16,16)
									//cv::Size(8,8), //blockStride, cv::Size(8,8)
									//cv::Size(8,8), //cellSize, cv::Size(8,8)
									//9); //nbins,
	cv::vector<float>		ders_torso;
	cv::vector<cv::Point>	locs_torso;

	hog_torso.compute(user->_features._gray_torso,ders_torso,cv::Size(32,32),cv::Size(0,0),locs_torso);

	if(ders_torso.size())
		user->_features._hog_vis_torso = get_hogdescriptor_visu(user, user->_features._gray_torso,ders_torso);
	

	////Calc Histograms
	////for(int i = 0 ; i < ders_torso.size() ; i+=9){
	//	for(int j = 0 ; j < 9 ; ++j){
	//		user->_features._hog_hist_torso[j] = 0;
	//	}
	////}
	//for(int i = 0 ; i < ders_torso.size() ; i+=9){
	//	for(int j = 0 ; j < 9; ++j){
	//		user->_features._hog_hist_torso[j] += ders_torso[i + j];
	//	}
	//}

	int n_bins = 10;
	int hist_w = 270; int hist_h = 200;
	int bin_w = cvRound( (double) hist_w/n_bins );
	

	user->_features._hog_vis_hist_torso = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

//	cv::normalize(user->_features._hog_hist_torso,user->_features._hog_hist_torso);

	float max = 0;
	int idx = 0;
	float temp[9];
	for(int j = 0 ; j < 9 ; ++j){
		temp[j] = user->_features._hog_hist_torso[j];
		if(user->_features._hog_hist_torso[j] > max){
			max = user->_features._hog_hist_torso[j];
			idx = j;
		}
	}

	if(idx){
		for(int j = 0 ; j < 9 ; ++j){
			user->_features._hog_hist_torso[j] = temp[(idx + j) % 9];
		}
	}

	for( int i = 1; i < n_bins; i++ )	{
		cv::line( user->_features._hog_vis_hist_torso, 
						cv::Point( bin_w*(i-1), hist_h - hist_h*user->_features._hog_hist_torso[i-1]),
						cv::Point( bin_w*(i)  , hist_h - hist_h*user->_features._hog_hist_torso[i]  ),
						cv::Scalar( 255, 0, 0), 2, 8, 0  );
	}

	return user->_features._hog_vis_torso.rows;
}

void show_hog(User* user, int kinect_id, int user_id){
	//char win_buff_hog_bbox[128];
	//sprintf(win_buff_hog_bbox,"Hog_bbox(%d)(%d)",kinect_id,user_id);
	//cv::imshow(win_buff_hog_bbox,user->_features._hog_vis_bbox);

	//char win_buff_hog_mask[128];
	//sprintf(win_buff_hog_mask,"Hog_mask(%d)(%d)",kinect_id,user_id);
	//cv::imshow(win_buff_hog_mask,user->_features._hog_vis_mask);

	char win_buff_hog_torso[128];
	sprintf(win_buff_hog_torso,"Hog_Torso(%d)(%d)",kinect_id,user_id);
	cv::imshow(win_buff_hog_torso,user->_features._hog_vis_torso);

	char win_buff_hog_hist_torso[128];
	sprintf(win_buff_hog_hist_torso,"Hog_Torso_Hist(%d)(%d)",kinect_id,user_id);
	cv::imshow(win_buff_hog_hist_torso,user->_features._hog_vis_hist_torso);

	char win_buff_hog_canny_torso[128];
	sprintf(win_buff_hog_canny_torso,"Hog_Torso_Canny(%d)(%d)",kinect_id,user_id);
	cv::imshow(win_buff_hog_canny_torso,user->_features._canny_torso);

	//char win_buff_hog_hist_bbox[128];
	//sprintf(win_buff_hog_hist_bbox,"Hog_bbox_Hist(%d)(%d)",kinect_id,user_id);
	//cv::imshow(win_buff_hog_hist_bbox,user->_features._hog_vis_hist_bbox);

	//char win_buff_hog_hist_mask[128];
	//sprintf(win_buff_hog_hist_mask,"Hog_mask_Hist(%d)(%d)",kinect_id,user_id);
	//cv::imshow(win_buff_hog_hist_mask,user->_features._hog_vis_hist_mask);
}

void show_body_parts(User* user, int kinect_id, int user_id){
	//char win_user_head[128];
	//sprintf(win_user_head,"UserHead(%d)(%d)",kinect_id,user_id);
	//cv::Mat head;
	//user->_color_full(user->_bbox).copyTo(head,user->_division3d._mask_head(user->_bbox));
	//cv::imshow(win_user_head,head);

	//char win_user_torso[128];
	//sprintf(win_user_torso,"UserTorso(%d)(%d)",kinect_id,user_id);
	//cv::Mat torso;
	//user->_color_full(user->_bbox).copyTo(torso,user->_division3d._mask_torso(user->_bbox));
	//cv::circle(torso,cv::Point(user->_centroid_2d.x - user->_bbox.x, user->_centroid_2d.y - user->_bbox.y),5,cv::Scalar(255,0,0),-1);
	//cv::imshow(win_user_torso,torso);

	//char win_user_legs[128];
	//sprintf(win_user_legs,"UserLegs(%d)(%d)",kinect_id,user_id);
	//cv::Mat legs;
	//user->_color_full(user->_bbox).copyTo(legs,user->_division3d._mask_legs(user->_bbox));
	//cv::imshow(win_user_legs,legs);


}

void show_histograms(User* user, int kinect_id, int user_id){
	//char win_buff_hist_rgb[128];
	//sprintf(win_buff_hist_rgb,"HistRGB(%d)(%d)",kinect_id,user_id);
	//cv::imshow(win_buff_hist_rgb,user->_histogram._histogram_rgb_vis_full);

	//char win_buff_hist_rgb_head[128];
	//sprintf(win_buff_hist_rgb,"HistRGB-Head(%d)(%d)",kinect_id,user_id);
	//cv::imshow(win_buff_hist_rgb,user->_histogram._histogram_rgb_vis_head);

	//char win_buff_hist_rgb_torso[128];
	//sprintf(win_buff_hist_rgb_torso,"HistRGB-Torso(%d)(%d)",kinect_id,user_id);
	//cv::imshow(win_buff_hist_rgb_torso,user->_histogram._histogram_rgb_vis_torso);

	//char win_buff_hist_rgb_legs[128];
	//sprintf(win_buff_hist_rgb_legs,"HistRGB-Legs(%d)(%d)",kinect_id,user_id);
	//cv::imshow(win_buff_hist_rgb_legs,user->_histogram._histogram_rgb_vis_legs);




	//char win_buff_hist_hsv[128];
	//sprintf(win_buff_hist_hsv,"HistHSV(%d)(%d)",kinect_id,user_id);
	//cv::imshow(win_buff_hist_hsv,user->_histogram._histogram_hsv_vis_full);

	//char win_buff_hist_hsv_head[128];
	//sprintf(win_buff_hist_hsv_head,"HistHSV-Head(%d)(%d)",kinect_id,user_id);
	//cv::imshow(win_buff_hist_hsv_head,user->_histogram._histogram_hsv_vis_head);

	//char win_buff_hist_hsv_torso[128];
	//sprintf(win_buff_hist_hsv_torso,"HistHSV-Torso(%d)(%d)",kinect_id,user_id);
	//cv::imshow(win_buff_hist_hsv_torso,user->_histogram._histogram_hsv_vis_torso);

	//char win_buff_hist_hsv_legs[128];
	//sprintf(win_buff_hist_hsv_legs,"HistHSV-Legs(%d)(%d)",kinect_id,user_id);
	//cv::imshow(win_buff_hist_hsv_legs,user->_histogram._histogram_hsv_vis_legs);



	//char win_user_full_rgb[128];
	//sprintf(win_user_full_rgb,"UserFullRGB(%d)(%d)",kinect_id,user_id);
	//cv::imshow(win_user_full_rgb,user->_histogram._hist_img_result_rgb);

	//char win_buff_hist_rgb[128];
	//sprintf(win_buff_hist_rgb,"HistRGB(%d)(%d)",kinect_id,user_id);
	//cv::imshow(win_buff_hist_rgb,user->_histogram._hist_vis_rgb);



	char win_user_full_hsv[128];
	sprintf(win_user_full_hsv,"UserFullHSV(%d)(%d)",kinect_id,user_id);
	cv::imshow(win_user_full_hsv,user->_histogram._hist_img_result_hsv_cvt);

	char win_buff_hist_hsv[128];
	sprintf(win_buff_hist_hsv,"HistHSV(%d)(%d)",kinect_id,user_id);
	cv::imshow(win_buff_hist_hsv,user->_histogram._hist_vis_hsv_cvt);
	
}

void compare_users(std::vector<User*>* users1, std::vector<User*>* users2){
	if(!users1->size() || !users2->size()) return;
	int text_step = 30;

	cv::Mat text_mat_histogram(300,700,CV_8UC3,cv::Scalar(255,255,255));
	int text_level_hist = 30;

	//cv::Mat text_mat_features(300,700,CV_8UC3,cv::Scalar(255,255,255));
	//int text_level_feat = 30;

	//cv::Mat text_mat_bio(300,700,CV_8UC3,cv::Scalar(255,255,255));
	//int text_level_bio = 30;
	
	
	//cv::putText(text_mat_histogram, "0-0|  Full  |  Head  | Torso |  Legs  |", cv::Point(0,text_level_hist), cv::FONT_HERSHEY_DUPLEX, 0.75, cv::Scalar(0,0,0), 0.75); text_level_hist+=text_step;

	for(int u1 = 0 ; u1 < users1->size() ; ++u1){
		User* user1 = users1->at(u1);

		for(int u2 = 0 ; u2 < users2->size() ; ++u2){
			User* user2 = users2->at(u2);
			/*
			{ //FEATURES
				std::vector<float> diff(9);

				for(int i = 0 ; i < 9 ; ++i){
					diff[i] = abs(user1->_features._hog_hist_mask[i] - user2->_features._hog_hist_mask[i]);
				}

				int n_bins = 9;
				int hist_w = 270; int hist_h = 200;
				int bin_w = cvRound( (double) hist_w/n_bins );
	
				cv::Mat diff_mat = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
			
				for( int i = 1; i < n_bins; i++ )	{
					cv::line(	diff_mat, 
								cv::Point( bin_w*(i-1), hist_h - hist_h*diff[i-1]),
								cv::Point( bin_w*(i)  , hist_h - hist_h*diff[i]  ),
								cv::Scalar( 0, 0, 255), 2, 8, 0  );
					cv::line(	diff_mat,
								cv::Point( bin_w*(i-1), hist_h - hist_h*user1->_features._hog_hist_mask[i-1]),
								cv::Point( bin_w*(i  ), hist_h - hist_h*user1->_features._hog_hist_mask[i  ]),
								cv::Scalar( 255, 0, 0), 2, 8, 0  );
					cv::line(	diff_mat, 
								cv::Point( bin_w*(i-1), hist_h - hist_h*user2->_features._hog_hist_mask[i-1]),
								cv::Point( bin_w*(i  ), hist_h - hist_h*user2->_features._hog_hist_mask[i  ]),
								cv::Scalar( 0, 255, 0), 2, 8, 0  );
				}

				int compare_method = 0;
				double compare_value = cv::compareHist(user1->_features._hog_hist_mask,user2->_features._hog_hist_mask,compare_method);

				char buff[128];
				sprintf(buff,"%.4f",compare_value);
				cv::putText(text_mat_features, buff,  cv::Point(0,text_level_feat), cv::FONT_HERSHEY_DUPLEX, 0.75, cv::Scalar(255,255,255), 0.75); text_level_feat+=text_step;
			}
			*/
			{//HISTOGRAMS / PARTS
				{//MY HSV
					int compare_method = 0;

					float compare_value = cv::compareHist(user1->_histogram._hist_hsv,user2->_histogram._hist_hsv,0);

					char buff[256];
					sprintf(buff,"%d-%d - %.4f",u1,u2,compare_value);
					cv::putText(text_mat_histogram, buff, cv::Point(0,text_level_hist), cv::FONT_HERSHEY_DUPLEX, 0.75, cv::Scalar(0,0,0), 0.75); text_level_hist+=text_step;

				}
				//{//HSV
				//	int compare_method = 0;

				//	double compare_value_full =	cv::compareHist(user1->_histogram._histogram_hsv_full, 
				//												user2->_histogram._histogram_hsv_full, compare_method );
				//	double compare_value_head =	cv::compareHist(user1->_histogram._histogram_hsv_head, 
				//												user2->_histogram._histogram_hsv_head, compare_method );
				//	double compare_value_torso=	cv::compareHist(user1->_histogram._histogram_hsv_torso, 
				//												user2->_histogram._histogram_hsv_torso, compare_method );
				//	double compare_value_legs =	cv::compareHist(user1->_histogram._histogram_hsv_legs, 
				//												user2->_histogram._histogram_hsv_legs, compare_method );

				//	char buff[256];
				//	sprintf(buff,"%d-%d| %.4f| %.4f| %.4f| %.4f|",u1,u2,compare_value_full,
				//														compare_value_head,
				//														compare_value_torso,
				//														compare_value_legs);
				//	cv::putText(text_mat_histogram, buff, cv::Point(0,text_level_hist), cv::FONT_HERSHEY_DUPLEX, 0.75, cv::Scalar(0,0,0), 0.75); text_level_hist+=text_step;
				//}
				//{//RGB
				//	int compare_method = 0;

				//	double compare_value_full =	cv::compareHist(user1->_histogram._histogram_rgb, 
				//												user2->_histogram._histogram_rgb, compare_method );
				//	char buff[256];
				//	sprintf(buff,"%d-%d| %.4f",u1,u2,compare_value_full);
				//	cv::putText(text_mat_histogram, buff, cv::Point(0,text_level_hist), cv::FONT_HERSHEY_DUPLEX, 0.75, cv::Scalar(0,0,0), 0.75); text_level_hist+=text_step;
				//}
			}

			//{//BIO
			//	{//HEIGHT
			//		float height1 = user1->_division3d._max_height;
			//		float height2 = user2->_division3d._max_height;

			//		float diff = abs(height1 - height2);

			//		char buff[128];
			//		sprintf(buff,"Height(%d) = %.2f",u1,height1);
			//		cv::putText(text_mat_bio, buff,  cv::Point(0,text_level_feat), cv::FONT_HERSHEY_DUPLEX, 0.75, cv::Scalar(255,255,255), 0.75); text_level_feat+=text_step;

			//		char buff2[128];
			//		sprintf(buff2,"Height(%d) = %.2f",u2,height2);
			//		cv::putText(text_mat_bio, buff2,  cv::Point(0,text_level_feat), cv::FONT_HERSHEY_DUPLEX, 0.75, cv::Scalar(255,255,255), 0.75); text_level_feat+=text_step;

			//		char buff3[128];
			//		sprintf(buff3,"Difference = %.2f",diff);
			//		cv::putText(text_mat_bio, buff3,  cv::Point(0,text_level_feat), cv::FONT_HERSHEY_DUPLEX, 0.75, cv::Scalar(255,255,255), 0.75); text_level_feat+=text_step;

			//	}
			//}
		}
	}

	cv::imshow("Text Histograms",text_mat_histogram);
	//cv::imshow("Text Features",text_mat_features);
	//cv::imshow("Text Bio",text_mat_bio);
}

}

int main_pcl_multi_tracking_3d(int argc, char* argv[]){
	NIKinect2Manager* kinect_manager = new NIKinect2Manager();

	int n_kinects = kinect_manager->initialize_all_kinects();

	std::vector<std::vector<User*>> users_kinect(n_kinects);
	int* users_kinect_ac = (int*)malloc(sizeof(int) * n_kinects);
	
	bool* flags_plane = (bool*)malloc(sizeof(bool) * n_kinects);
	std::vector<ToolBox::Plane*> floor_planes(n_kinects);
	std::vector<cv::Mat> floor_matrix(n_kinects);
	std::vector<cv::Mat> floor_inverse(n_kinects);

	for(int k = 0 ; k < n_kinects ; ++k){
		for(int i = 0 ; i < 8 ; ++i){
			users_kinect[k].push_back(new User());
		}
		users_kinect_ac[k] = 0;
		floor_planes[k] = new ToolBox::Plane();
		flags_plane[k] = false;

		if(BODY_PART_HD){
			kinect_manager->get_kinect(k)->set_color_hd(true);
		}
	}

	pcl::visualization::PCLVisualizer viewer("Simple cloud_file Viewer");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
	viewer.addCoordinateSystem (1.0);
	viewer.initCameraParameters ();

	int thresh_head = 270;

	cv::namedWindow("Controls");
	cv::createTrackbar("Head","Controls",&thresh_head,500);	
	cv::createTrackbar("canny","Controls",&lowThreshold, max_lowThreshold);
	cv::createTrackbar("zeros","Controls",&non_zeros_val, max_non_zeros);
	cv::createTrackbar("color","Controls",&thresh_rgb, 256);
	cv::createTrackbar("Black-V","Controls",&thresh_hsv_black_v, 255);
	cv::createTrackbar("White-S","Controls",&thresh_hsv_white_s, 255);
	cv::createTrackbar("White-V","Controls",&thresh_hsv_white_v, 255);

	char c = 0;
	while((c = cv::waitKey(11)) != 27){
		if(!kinect_manager->update_all())
			break;

		cv::Mat text_mat1(500,500,CV_8UC3,cv::Scalar(255,255,255));

		for(int k = 0 ; k < n_kinects ; ++k){
			cv::Mat color,depth,mask,user;

			NIKinect2* kinect = kinect_manager->get_kinect(k);

			if(kinect){
				kinect->get_depth_8(depth);
				kinect->get_color(color);
		
				//if(k == 0)
				//	color *= 0.5;
				//if(k == 1)
				//	color *= 1.75;
				std::vector<nite::UserData*>* users_data = kinect->get_users_data();
				std::vector<int> *users_ids = kinect->get_users_ids();

				if(users_data && users_ids){
					users_kinect_ac[k] = 0;
					for(int u = 0 ; u < users_ids->size() ; ++u){
						nite::UserData data;

						if(kinect->get_user_data(users_ids->at(u)-1,data) && data.getCenterOfMass().x &&
							data.getBoundingBox().max.y - data.getBoundingBox().min.y > 100){
							
							if(data.isNew()){
								printf("\n\nNEW USER (EXP)\n\n");
							}

							init_user(users_kinect[k][u], kinect, &data, color);

							users_kinect_ac[k]++;

							generate_3d(users_kinect[k][u]);

							cv::Mat temp_user;
							users_kinect[k][u]->_color_full.copyTo(temp_user,users_kinect[k][u]->_mask_full);
							cv::circle(temp_user,users_kinect[k][u]->_centroid_2d,7,cv::Scalar(0,0,255),-1);

							//if(calc_hog(users_kinect[k][u])){
							//	show_hog(users_kinect[k][u],k,u);
							//}

							if(floor_planes[k]){
								//BODY PARTS
								{
									generate_user_parts(users_kinect[k][u],floor_planes[k],thresh_head);
									
									//calc_center_rect(users_kinect[k][u]);
									//if(calc_center_canny(users_kinect[k][u])){
									//	//if(calc_center_hog(users_kinect[k][u])){
									//	//	show_hog(users_kinect[k][u],k,u);
									//	//}
									//}
									
									//show_body_parts(users_kinect[k][u],k,u);

									

									char buff_max[128];
									char buff_min[128];

									sprintf(buff_max,"Max:%.4f",users_kinect[k][u]->_division3d._max_height);
									sprintf(buff_min,"Min:%.4f",users_kinect[k][u]->_division3d._min_height);

									cv::putText(temp_user, buff_max, cv::Point(320,30) , cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255,255,255), 0.75);
									cv::putText(temp_user, buff_min, cv::Point(320,60), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255,255,255), 0.75);
								}

								//HISTOGRAMS
								{
									hsv_histogram2(users_kinect[k][u]);

									//rgb_histogram2(users_kinect[k][u]);
																
									show_histograms(users_kinect[k][u],k,u);
								}								
							}
						}
					}
				}

				char color_win[128];
				sprintf(color_win,"Color(%d)",k);
				cv::imshow(color_win,color);
			} //Kinect

			if(c == 'f'){
				if(flags_plane[k]){
					flags_plane[k] = false;
				}
				else{
					if(kinect->calc_floor_plane()){
						nite::Plane* plane = kinect->get_floor_plane();

						if(plane){
							double plane_a = plane->normal.x;
							double plane_b = plane->normal.y;
							double plane_c = plane->normal.z;
							double plane_d = -(	plane->normal.x*plane->point.x + 
												plane->normal.y*plane->point.y + 
												plane->normal.z*plane->point.z);

							floor_planes[k]->set(plane_a,plane_b,plane_c,plane_d);

							cv::Vec3f vec_1(1,0,0);
							cv::Vec3f vec_2 = vec_1.cross(cv::Vec3f(plane_a,plane_b,plane_c));

							floor_matrix[k].create(3,3,CV_64FC1);
							floor_matrix[k].ptr<double>(0)[0] = vec_1.val[0];
							floor_matrix[k].ptr<double>(0)[1] = vec_1.val[1];
							floor_matrix[k].ptr<double>(0)[2] = vec_1.val[2];
							floor_matrix[k].ptr<double>(1)[0] = -plane_a;
							floor_matrix[k].ptr<double>(1)[1] = -plane_b;
							floor_matrix[k].ptr<double>(1)[2] = -plane_c;
							floor_matrix[k].ptr<double>(2)[0] = vec_2.val[0];
							floor_matrix[k].ptr<double>(2)[1] = vec_2.val[1];
							floor_matrix[k].ptr<double>(2)[2] = vec_2.val[2];

							floor_inverse[k] = floor_matrix[k].inv(1);

							flags_plane[k] = true;
						}
					}
				}
			}
		} //N Kinects

		std::vector<User*> temp_users;
		std::vector<User*> users_kinect_1;
		std::vector<User*> users_kinect_2;

		for(int i = 0 ; i < users_kinect_ac[0] ; ++i){
			temp_users.push_back(users_kinect[0][i]);
			users_kinect_1.push_back(users_kinect[0][i]);
		}
		for(int i = 0 ; i < users_kinect_ac[1] ; ++i){
			temp_users.push_back(users_kinect[1][i]);
			users_kinect_2.push_back(users_kinect[1][i]);
		}

		//cv::imshow("Text1",text_mat1);
		
		compare_users(&users_kinect_1, &users_kinect_2);

		//if(temp_users.size() == 2){
		//	int compare_method = 0;
		//	double compare_value = compareHist( temp_users[0]->_histogram._histogram_rgb, 
		//										temp_users[1]->_histogram._histogram_rgb, compare_method );

		//	char buff[128];
		//	sprintf(buff,"Compare:%.4f",compare_value);
		//	cv::putText(text_mat1, buff, cv::Point(30,30) , cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0,0,0), 0.75);
		//	
		//}

		

		//Reset Points
		if(temp_users.size()){
			//cloud.points.clear();
			viewer.removeAllPointClouds();

			for(int u = 0 ; u < temp_users.size() ; ++u){
				char cloud_name[128];
		//		char cloud_head_name[128];
		//		char cloud_torso_name[128];
		//		char cloud_leg_name[128];
				sprintf(cloud_name,"Cloud%d",u);
		//		sprintf(cloud_head_name,"CloudHead%d",u);
		//		sprintf(cloud_torso_name,"CloudTorso%d",u);
		//		sprintf(cloud_leg_name,"CloudLeg%d",u);

				viewer.addPointCloud(temp_users[u]->_division3d._cloud.makeShared(),cloud_name);
		//		viewer.addPointCloud(temp_users[u]->_cloud_head.makeShared(),cloud_head_name);
		//		viewer.addPointCloud(temp_users[u]->_cloud_torso.makeShared(),cloud_torso_name);
		//		viewer.addPointCloud(temp_users[u]->_cloud_legs.makeShared(),cloud_leg_name);
			}
		//	
			viewer.spinOnce(50);
		}
	} //While
	kinect_manager->~NIKinect2Manager();

	return 0;
}