#include "MultiTrackingColor.h"

#include <NIKinect2Manager.h>

namespace{
class User{
	public:
		User()/*:_mask(480,640,CV_8UC1)*/{};
		User(User* user){
			this->_bbox = user->_bbox;
			this->_centroid_3d = user->_centroid_3d;
			this->_centroid_2d = user->_centroid_2d;

			user->_mask.copyTo(this->_mask);

			user->_color_masked.copyTo(this->_color_masked);
			user->_color_masked_hsv.copyTo(this->_color_masked_hsv);

			user->_user.copyTo(this->_user);
			user->_user_mask.copyTo(this->_user_mask);
			user->_user_hsv.copyTo(this->_user_hsv);

			user->_histogram_rgb.copyTo(this->_histogram_rgb);
			user->_histogram_rgb_r.copyTo(this->_histogram_rgb_r);
			user->_histogram_rgb_g.copyTo(this->_histogram_rgb_g);
			user->_histogram_rgb_b.copyTo(this->_histogram_rgb_b);
			user->_histogram_hsv.copyTo(this->_histogram_hsv);
			user->_histogram_hsv_h.copyTo(this->_histogram_hsv_h);
			user->_histogram_hsv_s.copyTo(this->_histogram_hsv_s);
			user->_histogram_hsv_v.copyTo(this->_histogram_hsv_v);

			user->_hsv_hist_h.copyTo(this->_hsv_hist_h);
			user->_hsv_hist_s.copyTo(this->_hsv_hist_s);
			user->_hsv_hist_v.copyTo(this->_hsv_hist_v);

			user->_pseudo_h.copyTo(this->_pseudo_h);
			user->_pseudo_s.copyTo(this->_pseudo_s);

			user->_mask1.copyTo(this->_mask1);
			user->_mask2.copyTo(this->_mask2);
			user->_mask3.copyTo(this->_mask3);
		}
		~User(){};

		cv::Rect	_bbox;
		cv::Point3f	_centroid_3d;
		cv::Point	_centroid_2d;

		cv::Mat _mask;

		cv::Mat _color_masked;
		cv::Mat _color_masked_hsv;

		cv::Mat _user;
		cv::Mat _user_mask;
		cv::Mat _user_hsv;

		cv::Mat _histogram_rgb;
		cv::Mat _histogram_rgb_r;
		cv::Mat _histogram_rgb_g;
		cv::Mat _histogram_rgb_b;
		cv::Mat _histogram_hsv;
		cv::Mat _histogram_hsv_h;
		cv::Mat _histogram_hsv_s;
		cv::Mat _histogram_hsv_v;

		cv::Mat _hsv_hist_h;
		cv::Mat _hsv_hist_s;
		cv::Mat _hsv_hist_v;

		cv::Mat _pseudo_h;
		cv::Mat _pseudo_s;

		cv::Mat _mask1;
		cv::Mat _mask2;
		cv::Mat _mask3;

		int _id;
		double _conf;
	};

int n_images = 3;
std::vector<cv::Mat> mat_color;
std::vector<cv::Mat> mat_mask;
std::vector<cv::Mat> mat_color_masked;
std::vector<cv::Mat> mat_hsv;
std::vector<cv::Mat> mat_hsv_mask;

void init_user(User* user, NIKinect2* kinect, nite::UserData* data, cv::Mat color){
	if(!user || !kinect || !data || !color.rows || !color.cols) return;

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

	kinect->get_user_mask(data->getId(),user->_mask);

	cv::dilate(user->_mask,user->_mask,cv::Mat(3,3,CV_8UC1));
	cv::erode(user->_mask,user->_mask,cv::Mat(7,7,CV_8UC1));

	color.copyTo(user->_color_masked,user->_mask);
	cv::cvtColor(color,user->_color_masked_hsv,CV_RGB2HLS);

	color(user->_bbox).copyTo(user->_user);
	user->_mask(user->_bbox).copyTo(user->_user_mask);
}

void split_user(User *user){
	cv::Rect bbox1(user->_bbox);
	bbox1.height /=2;
	cv::Rect bbox2(user->_bbox);
	bbox2.height /=2;
	bbox2.y += bbox2.height;

	user->_color_masked(bbox1).copyTo(user->_mask1);
	user->_color_masked(bbox2).copyTo(user->_mask2);
}

void rgb_histogram(User* user, int kinect, int user_id){
	// Separate the image in 3 places ( B, G and R )
	cv::vector<cv::Mat> bgr_planes;
	cv::split( user->_user, bgr_planes );

	/// Establish the number of bins
	int histSize = 256;

	/// Set the ranges ( for B,G,R) )
	float range[] = { 0, 256 } ;
	const float* histRange = { range };

	bool uniform = true; bool accumulate = false;
	int channels[] = { 0, 1 ,2};
	//cv::Mat b_hist, g_hist, r_hist;

	/// Compute the histograms:
	//cv::calcHist( &user->_user, 1, channels, user->_user_mask, user->_histogram_rgb, 3, &histSize, &histRange, uniform, accumulate );
	cv::calcHist( &bgr_planes[0], 1, 0, user->_user_mask, user->_histogram_rgb_b, 1, &histSize, &histRange, uniform, accumulate );
	cv::calcHist( &bgr_planes[1], 1, 0, user->_user_mask, user->_histogram_rgb_g, 1, &histSize, &histRange, uniform, accumulate );
	cv::calcHist( &bgr_planes[2], 1, 0, user->_user_mask, user->_histogram_rgb_r, 1, &histSize, &histRange, uniform, accumulate );

	// Draw the histograms for B, G and R
	int hist_w = 512; int hist_h = 400;
	int bin_w = cvRound( (double) hist_w/histSize );

	cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

	/// Normalize the result to [ 0, histImage.rows ]
	//cv::normalize(user->_histogram_rgb, user->_histogram_rgb, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(user->_histogram_rgb_b, user->_histogram_rgb_b, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(user->_histogram_rgb_g, user->_histogram_rgb_g, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(user->_histogram_rgb_r, user->_histogram_rgb_r, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

	///// Draw for each channel
	//for( int i = 1; i < histSize; i++ )
	//{
	//	cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram_rgb_b.at<float>(i-1)) ) ,
	//					cv::Point( bin_w*(i), hist_h - cvRound(user->_histogram_rgb_b.at<float>(i)) ),
	//					cv::Scalar( 255, 0, 0), 2, 8, 0  );
	//	cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram_rgb_g.at<float>(i-1)) ) ,
	//					cv::Point( bin_w*(i), hist_h - cvRound(user->_histogram_rgb_g.at<float>(i)) ),
	//					cv::Scalar( 0, 255, 0), 2, 8, 0  );
	//	cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram_rgb_r.at<float>(i-1)) ) ,
	//					cv::Point( bin_w*(i), hist_h - cvRound(user->_histogram_rgb_r.at<float>(i)) ),
	//					cv::Scalar( 0, 0, 255), 2, 8, 0  );
	//	//cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram_rgb.at<float>(i-1)) ) ,
	//	//				cv::Point( bin_w*(i), hist_h - cvRound(user->_histogram_rgb.at<float>(i)) ),
	//	//				cv::Scalar( 0, 255, 255), 2, 8, 0  );
	//}

	//char win_hist[128];
	//char win_user[128];
	//sprintf(win_hist,"RGB Hist(%d-%d)",kinect,user_id);
	//sprintf(win_user,"RGB User (%d-%d)",kinect,user_id);
	//cv::imshow(win_hist, histImage );
	//cv::imshow(win_user, user->_user );
}

void hsv_histogram(User* user, int kinect, int user_id){

	cv::cvtColor( user->_user, user->_user_hsv, CV_BGR2HSV );

	cv::vector<cv::Mat> hsv_planes;
	cv::split( user->_user_hsv, hsv_planes );

	/// Using 30 bins for hue and 32 for saturation
	int h_bins = 10; int s_bins = 10;
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

	user->_hsv_hist_h = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
	user->_hsv_hist_s = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

	cv::calcHist( &hsv_planes[0], 1, 0, user->_user_mask, user->_histogram_hsv_h, 1, &h_bins, &h_ranges, uniform, accumulate );
	cv::calcHist( &hsv_planes[1], 1, 0, user->_user_mask, user->_histogram_hsv_s, 1, &h_bins, &s_ranges, uniform, accumulate );
	//cv::calcHist( &hsv_planes[2], 1, 0, user->_user_mask, user->_histogram_hsv_v, 1, &histSize, &histRange, uniform, accumulate );
	cv::normalize(user->_histogram_hsv_h, user->_histogram_hsv_h, 0, user->_hsv_hist_h.rows, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(user->_histogram_hsv_s, user->_histogram_hsv_s, 0, user->_hsv_hist_h.rows, cv::NORM_MINMAX, -1, cv::Mat() );
	//cv::normalize(user->_histogram_hsv_v, user->_histogram_rgb_r, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

	/// Calculate the histograms for the HSV images
	cv::calcHist( &user->_user_hsv, 1, channels, user->_user_mask, user->_histogram_hsv, 2, histSize, ranges, true, false );
	cv::normalize( user->_histogram_hsv, user->_histogram_hsv, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

	cv::applyColorMap(hsv_planes[0],user->_pseudo_h,9);
	cv::applyColorMap(hsv_planes[1],user->_pseudo_s,9);

	//cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

	// Draw for each channel
	for( int i = 1; i < h_bins; i++ )	{
		cv::line( user->_hsv_hist_h, cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram_hsv_h.at<float>(i-1)) ) ,
						cv::Point( bin_w*(i), hist_h - cvRound(user->_histogram_hsv_h.at<float>(i)) ),
						cv::Scalar( 255, 0, 0), 2, 8, 0  );
		cv::line( user->_hsv_hist_s, cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram_hsv_s.at<float>(i-1)) ) ,
						cv::Point( bin_w*(i), hist_h - cvRound(user->_histogram_hsv_s.at<float>(i)) ),
						cv::Scalar( 0, 255, 0), 2, 8, 0  );
		//cv::line( user->_hsv_hist_v, cv::Point( bin_w*(i-1), hist_h - cvRound(user->_histogram_rgb_r.at<float>(i-1)) ) ,
		//				cv::Point( bin_w*(i), hist_h - cvRound(user->_histogram_rgb_r.at<float>(i)) ),
		//				cv::Scalar( 0, 0, 255), 2, 8, 0  );
	}

	//for(int i = 0 ; i < n_images ; ++i){
	//	cv::MatND hist_rec;

	//	cv::calcHist( &mat_hsv[i], 1, channels, mat_mask[i], hist_rec, 2, histSize, ranges, true, false );
	//	cv::normalize( hist_rec, hist_rec, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

	//	int compare_method = 0;
	//	double compare_value = compareHist( user->_histogram_hsv, hist_rec, compare_method );

	//	printf("%.4f\t",compare_value);

	//	//char win_comp[128];
	//	//sprintf(win_comp,"HSV compare (%d)",i);
	//	//cv::imshow(win_comp, mat_hsv_mask[i] );
	//}
	//printf("\n");

	//char win_user[128];
	//sprintf(win_user,"HSV User (%d-%d)",kinect,user_id);
	//cv::Mat masked;
	//user->_user_hsv.copyTo(masked,user->_user_mask);
	//cv::imshow(win_user, masked );

}

void pseudo_color(User* user, int kinect, int user_id){
	cv::vector<cv::Mat> hsv_planes;
	cv::split( user->_user_hsv, hsv_planes );

	cv::Mat pseudo_color_h;
	cv::Mat pseudo_color_s;
	cv::Mat pseudo_color_v;

	cv::applyColorMap(hsv_planes[0],pseudo_color_h,9);
	cv::applyColorMap(hsv_planes[1],pseudo_color_s,9);
	cv::applyColorMap(hsv_planes[2],pseudo_color_v,9);

	char win_user_h[128];
	char win_user_s[128];
	char win_user_v[128];
	
	sprintf(win_user_h,"pseudo_color_h (%d-%d)",kinect,user_id);
	sprintf(win_user_s,"pseudo_color_s (%d-%d)",kinect,user_id);
	sprintf(win_user_v,"pseudo_color_v (%d-%d)",kinect,user_id);

	cv::imshow(win_user_h,pseudo_color_h);
	cv::imshow(win_user_s,pseudo_color_s);
	cv::imshow(win_user_v,pseudo_color_v);
}

void pseudo_color_comp(User* user1, User* user2, int kinect, int user_id){
	cv::vector<cv::Mat> hsv_planes1;
	cv::split( user1->_user_hsv, hsv_planes1 );

	cv::vector<cv::Mat> hsv_planes2;
	cv::split( user2->_user_hsv, hsv_planes2 );

	cv::Mat diff_h; cv::absdiff(hsv_planes1[0], hsv_planes2[0], diff_h);
	cv::Mat diff_s; cv::absdiff(hsv_planes1[1], hsv_planes2[1], diff_s);
	cv::Mat diff_v; cv::absdiff(hsv_planes1[2], hsv_planes2[2], diff_v);
	
	uchar* data = (uchar*)diff_h.data;

	for(int i = 0 ; i < diff_h.rows * diff_h.cols ; ++i){
		if(data[i] > 90){
			data[i] = 180 - data[i];
		}
	}

	uchar* data_s = (uchar*)diff_s.data;
	uchar* data_v = (uchar*)diff_v.data;
	for(int i = 0 ; i < diff_h.rows * diff_h.cols ; ++i){
		if(data[i] > 30)
			data[i] = 30;

		if(data_s[i] > 30)
			data_s[i] = 30;

		if(data_v[i] > 50)
			data_v[i] = 50;
	}

	cv::normalize(diff_h,diff_h,0,255,CV_MINMAX);
	cv::normalize(diff_s,diff_s,0,255,CV_MINMAX);
	cv::normalize(diff_v,diff_v,0,255,CV_MINMAX);

	cv::Mat pseudo_color_h;
	cv::Mat pseudo_color_s;
	cv::Mat pseudo_color_v;

	cv::applyColorMap(diff_h,pseudo_color_h,9);
	cv::applyColorMap(diff_s,pseudo_color_s,9);
	cv::applyColorMap(diff_v,pseudo_color_v,9);

	//char win_user_h[128];
	//char win_user_s[128];
	//char win_user_v[128];
	//
	//sprintf(win_user_h,"pseudo_color_h (%d-%d)",kinect,user_id);
	//sprintf(win_user_s,"pseudo_color_s (%d-%d)",kinect,user_id);
	//sprintf(win_user_v,"pseudo_color_v (%d-%d)",kinect,user_id);

	cv::imshow("Diff H",pseudo_color_h);
	cv::imshow("Diff S",pseudo_color_s);
	cv::imshow("Diff V",pseudo_color_v);
}

void load_images(){
	for(int i = 0 ; i < n_images ; ++i){
		char buff_color[128];
		char buff_depth[128];
		sprintf(buff_color,"Track\\Recordings\\%d_color.png",i);
		sprintf(buff_depth,"Track\\Recordings\\%d_depth.png",i);

		mat_color.push_back(cv::imread(buff_color));
		mat_mask.push_back(cv::imread(buff_depth,0));
		
		cv::Mat color_masked;
		mat_color[i].copyTo(color_masked,mat_mask[i]);
		mat_color_masked.push_back(cv::Mat(color_masked));

		cv::Mat hsv,hsv_masked;
		cv::cvtColor( mat_color[i], hsv, CV_BGR2HSV );
		mat_hsv.push_back(cv::Mat(hsv));

		hsv.copyTo(hsv_masked,mat_mask[i]);
		mat_hsv_mask.push_back(hsv_masked);

		//cv::imshow("color",mat_color[i]);
		//cv::imshow("mask",mat_mask[i]);
		//cv::imshow("color_masked",mat_color_masked[i]);
		//cv::imshow("hsv",mat_hsv[i]);
		//cv::imshow("hsv_masked",mat_hsv_mask[i]);
		//cv::waitKey();
	}
}

void write_text_on_image(cv::Mat& image, char* text, cv::Point position, cv::Scalar color = cv::Scalar(0,0,0)){
	if(!text) return;

	cv::putText(image, text, position, cv::FONT_HERSHEY_DUPLEX, 1, color, 0.75);
}

}

int main_multi_tracking_color(int argc, char* argv[]){
	NIKinect2Manager* kinect_manager = new NIKinect2Manager();

	NIKinect2::ni_initialize();

	int n_kinects = kinect_manager->initialize_all_kinects();

	//NIKinect2* kinect = new NIKinect2();
	//kinect->initialize("Track\\track1.oni");
	//kinect->enable_depth_generator();
	//kinect->enable_color_generator();
	//kinect->enable_user_generator();

	//kinect_manager->add_kinect(kinect);
	//n_kinects++;

	load_images();
	std::vector<std::vector<User*>> users_kinect(2);
	for(int i = 0 ; i < 10 ; ++i){
		for(int j = 0 ; j < 2 ; ++j){
			users_kinect[j].push_back(new User());
		}
	}
	int users_ac[2] = {0,0};

	double table_compare[200][16*4];
	int table_best_match[16*4];
	double table_best_match_conf[16*4];
	bool table_match_flag[200][16*4];
	for(int i = 0 ; i < 200 ; ++i){
		for(int j = 0 ; j < 16*4 ; ++j){
			table_compare[i][j] = 0.0;
			table_best_match[j] = -1;
			table_best_match_conf[j] = 0.0;
			table_match_flag[i][j] = false;
		}
	}

	std::vector<User*> users;

	cv::namedWindow("Text1");
	cv::namedWindow("Text2");
	cv::moveWindow("Text1",1280+0,0);
	cv::moveWindow("Text2",1280+640,0);

	cv::namedWindow("Color(0)");
	cv::namedWindow("Color(1)");
	cv::moveWindow("Color(0)",1280+0,1024-560);
	cv::moveWindow("Color(1)",1280+640,1024-560);

	//cv::namedWindow("Saturation(0)");
	//cv::namedWindow("Saturation(1)");

	//cv::moveWindow("Saturation(0)",1280,0);
	//cv::moveWindow("Saturation(1)",1920,0);

	//int saturation0 = 90;
	//int saturation1 = 90;
	//int hue0 = 90;
	//int hue1 = 90;
	//int value0 = 90;
	//int value1 = 90;
	//cv::createTrackbar("Sat","Saturation(0)",&saturation0,180);
	//cv::createTrackbar("Sat","Saturation(1)",&saturation1,180);
	//cv::createTrackbar("Hue","Saturation(0)",&hue0,180);
	//cv::createTrackbar("Hue","Saturation(1)",&hue1,180);
	//cv::createTrackbar("Value","Saturation(0)",&value0,180);
	//cv::createTrackbar("Value","Saturation(1)",&value1,180);

	//for(int i = 0 ; i < n_kinects ; ++i){
	//	NIKinect2* kinect = kinect_manager->get_kinect(i);
	//		
	//	openni::CameraSettings* settings = kinect->get_color_stream()->getCameraSettings();

	//	bool valid = settings->isValid();

	//	openni::Status st = settings->setAutoExposureEnabled(false);
	//	st = settings->setAutoWhiteBalanceEnabled(false);

	//	int exp = settings->getExposure();
	//	int gain = settings->getGain();

	//	st = settings->setExposure(50);
	//	st = settings->setGain(50);
	//	printf("");
	//}

	char c = 0;
	while((c = cv::waitKey(11)) != 27){
		if(!kinect_manager->update_all())
			break;

		cv::Mat text_mat1(500,500,CV_8UC3,cv::Scalar(255,255,255));
		cv::Mat text_mat2(500,500,CV_8UC3,cv::Scalar(255,255,255));
		int text_level1 = 30;
		int text_level2 = 30;
		int text_step = 30;

		for(int i = 0 ; i < n_kinects ; ++i){
			cv::Mat color,depth,mask;

			NIKinect2* kinect = kinect_manager->get_kinect(i);
			
			//openni::CameraSettings* settings = kinect->get_color_stream()->getCameraSettings();
			//settings->setAutoExposureEnabled(false);

			//int exp = settings->getExposure();
			//int gain = settings->getGain();

			//openni::Status st = settings->setExposure(60);

			if(kinect){
				kinect->get_depth_8(depth);
				kinect->get_color(color);
				kinect->get_users_map(mask);

				users_ac[i] = 0;

				std::vector<nite::UserData*>* users_data = kinect->get_users_data();
				std::vector<int> *users_ids = kinect->get_users_ids();

				if(users_data && users_ids){
					//printf("Users: ");
					for(int k = 0 ; k < users_ids->size() ; ++k){
						nite::UserData data;

						if(kinect->get_user_data(users_ids->at(k),data) && data.getCenterOfMass().x && 
							(data.getBoundingBox().max.y - data.getBoundingBox().min.y) > 100 &&
							(data.getBoundingBox().max.x - data.getBoundingBox().min.x) > 50){
							//User user;
							init_user(users_kinect[i][k], kinect, &data, color);

							cv::Mat just_user;
							cv::Mat user_mask;
							//user._color_masked(user._bbox).copyTo(just_user);
							color(users_kinect[i][k]->_bbox).copyTo(just_user);
							users_kinect[i][k]->_mask(users_kinect[i][k]->_bbox).copyTo(user_mask);

							rgb_histogram(users_kinect[i][k],i,k);

							hsv_histogram(users_kinect[i][k],i,k);

							//pseudo_color(users_kinect[i][k],i,k);

							users_ac[i]++;

							if(c == 'c'){
								static int ac = 0;

								char buff_color[128];
								char buff_depth[128];
								sprintf(buff_color,"Track\\Recordings\\%d_color.png",ac);
								sprintf(buff_depth,"Track\\Recordings\\%d_depth.png",ac);
								cv::Mat rec_color,rec_depth;

								color(users_kinect[i][k]->_bbox).copyTo(rec_color);
								users_kinect[i][k]->_mask(users_kinect[i][k]->_bbox).copyTo(rec_depth);

								cv::imwrite(buff_color,rec_color);
								cv::imwrite(buff_depth,rec_depth);
								ac++;
							}

							cv::circle(color,users_kinect[i][k]->_centroid_2d,7,cv::Scalar(0,0,255),-1);
							cv::rectangle(color,users_kinect[i][k]->_bbox,cv::Scalar(0,255,0),3);
							//hsv_histogram(just_user,i,k);

							//split_user(user);
							//
							char win_users[128];
							//char win_users_mask1[128];
							//char win_users_mask2[128];
							sprintf(win_users,"Users(%d) ID(%d)",i,data.getId());
							//sprintf(win_users_mask1,"Users(%d) ID(%d) Mask1",i,data.getId());
							//sprintf(win_users_mask2,"Users(%d) ID(%d) Mask2",i,data.getId());
							cv::imshow(win_users,users_kinect[i][k]->_user);
							//cv::imshow(win_users_mask1,user->_mask1);
							//cv::imshow(win_users_mask2,user->_mask2);
							
							char win_buff_hist_h[128];
							char win_buff_hist_s[128];
							
							sprintf(win_buff_hist_h,"Hist_H(%d)(%d)",i,data.getId());
							sprintf(win_buff_hist_s,"Hist_S(%d)(%d)",i,data.getId());
							
							cv::imshow(win_buff_hist_h,users_kinect[i][k]->_hsv_hist_h);
							cv::imshow(win_buff_hist_s,users_kinect[i][k]->_hsv_hist_s);


							char win_buff_pseudo_h[128];
							char win_buff_pseudo_s[128];

							sprintf(win_buff_pseudo_h,"Pseudo_H(%d)(%d)",i,data.getId());
							sprintf(win_buff_pseudo_s,"Pseudo_S(%d)(%d)",i,data.getId());

							cv::imshow(win_buff_pseudo_h,users_kinect[i][k]->_pseudo_h);
							cv::imshow(win_buff_pseudo_s,users_kinect[i][k]->_pseudo_s);

						}
					}
				}

				char win_color[128];
				char win_depth[128];
				
				sprintf(win_color,"Color(%d)",i);
				sprintf(win_depth,"Depth(%d)",i);
				
				cv::imshow(win_color,color);
				//cv::imshow(win_depth,depth);
			}
		}

		std::vector<User*> temp_users;
		for(int i = 0 ; i < users_ac[0] ; ++i){
			temp_users.push_back(users_kinect[0][i]);
		}
		for(int i = 0 ; i < users_ac[1] ; ++i){
			temp_users.push_back(users_kinect[1][i]);
		}

		for(int i = 0 ; i < temp_users.size() ; ++i){
			if(!users.size()){
				users.push_back(new User(temp_users[i]));
				goto end;
			}
			else{
				for(int j = 0 ; j < users.size() ; ++j){
					int compare_method = 0;
					double compare_value = compareHist( temp_users[i]->_histogram_hsv, 
														users[j]->_histogram_hsv, compare_method );

					table_compare[j][i] = compare_value;
					table_match_flag[j][i] = false;
					table_best_match[j] = -1;

					char win_buff_user[128];
					char win_buff_hist_h[128];
					char win_buff_hist_s[128];
					char win_buff_pseudo_h[128];
					char win_buff_pseudo_s[128];

					sprintf(win_buff_user,"User(%d)",j);
					sprintf(win_buff_hist_h,"Hist_H(%d)",j);
					sprintf(win_buff_hist_s,"Hist_S(%d)",j);
					sprintf(win_buff_pseudo_h,"Pseudo_H(%d)",j);
					sprintf(win_buff_pseudo_s,"Pseudo_S(%d)",j);

					cv::imshow(win_buff_user,users[j]->_user);
					cv::imshow(win_buff_hist_h,users[j]->_hsv_hist_h);
					cv::imshow(win_buff_hist_s,users[j]->_hsv_hist_s);
					cv::imshow(win_buff_pseudo_h,users[j]->_pseudo_h);
					cv::imshow(win_buff_pseudo_s,users[j]->_pseudo_s);
				}
			}
		}
		
		for(int i = 0 ; i < temp_users.size() ; ++i){
			int best_idx = -1;
			double max_val = 0;

			for(int j = 0 ; j < users.size() ; ++j){
				if(table_compare[j][i] > max_val){
					max_val = table_compare[j][i];
					best_idx = j;
				}

				char buff[128];
				sprintf(buff,"T(%d)S(%d)-(%.2f)",i,j, table_compare[j][i]);
				write_text_on_image(text_mat2,buff,cv::Point(0,text_level2));
				text_level2 += text_step;
			}

			if(best_idx > -1){
				table_best_match[i] = best_idx;
				table_best_match_conf[i] = max_val;

				
			}
		}

		for(int i = 0 ; i < temp_users.size() ; ++i){
			if(table_best_match_conf[i] > 0.8){
				temp_users[i]->_id = table_best_match[i];
				temp_users[i]->_conf = 100 + table_best_match_conf[i];
			} else
			if(table_best_match_conf[i] > 0.2){
				temp_users[i]->_id = table_best_match[i];
				temp_users[i]->_conf = table_best_match_conf[i];
			} else{
				users.push_back(new User(temp_users[i]));
				temp_users[i]->_id = users.size() -1;
				temp_users[i]->_conf = -1;//table_best_match_conf[i];
			}
		}

		for(int i = 0 ; i < temp_users.size() ; ++i){
			char buff[128];
			sprintf(buff,"TU(%d) - U(%d) - (%.2f)",i,temp_users[i]->_id, temp_users[i]->_conf);
			write_text_on_image(text_mat1,buff,cv::Point(0,text_level1));
			text_level1 += text_step;
		}


	end:
		cv::imshow("Text1",text_mat1);
		cv::imshow("Text2",text_mat2);


		//cv::Mat hsv0,color0,color0s;
		//cv::Mat hsv1,color1,color1s;


		//NIKinect2* kinect0 = kinect_manager->get_kinect(0);
		//NIKinect2* kinect1 = kinect_manager->get_kinect(1);
		//kinect0->get_color(color0);
		//kinect1->get_color(color1);

		//cv::cvtColor( color0, hsv0, CV_BGR2HSV );
		//cv::cvtColor( color1, hsv1, CV_BGR2HSV );

		//for (int i=0; i < hsv0.rows ; i++){
		//	for(int j=0; j < hsv0.cols; j++){
		//		// You need to check this, but I think index 1 is for saturation, but it might be 0 or 2
		//		int idx = 1;
		//		cv::Vec3b v = hsv0.at<cv::Vec3b>(i,j)[idx];

		//		hsv0.at<cv::Vec3b>(i,j)[0] += hue0 - 90;
		//		hsv1.at<cv::Vec3b>(i,j)[0] += hue1 - 90;

		//		hsv0.at<cv::Vec3b>(i,j)[1] += saturation0 - 90;
		//		hsv1.at<cv::Vec3b>(i,j)[1] += saturation1 - 90;

		//		hsv0.at<cv::Vec3b>(i,j)[2] += value0 - 90;
		//		hsv1.at<cv::Vec3b>(i,j)[2] += value1 - 90;

		//		// or:
		//		// img.at<cv::Vec3b>(i,j)[idx] += adds_constant_value;
		//	}
		//}

		//cv::cvtColor( hsv0, color0s, CV_HSV2BGR);
		//cv::cvtColor( hsv1, color1s, CV_HSV2BGR);

		//cv::imshow("Saturation(0)",color0s);
		//cv::imshow("Saturation(1)",color1s);

	} //while

	kinect_manager->~NIKinect2Manager();

	return 0;
}