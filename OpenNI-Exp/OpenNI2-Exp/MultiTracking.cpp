#include "MultiTracking.h"

#include <NIKinect2Manager.h>

namespace{
class User{
	public:
		User()/*:_mask(480,640,CV_8UC1)*/{};
		~User(){};

		cv::Rect	_bbox;
		cv::Point3f	_centroid_3d;
		cv::Point	_centroid_2d;

		cv::Mat _mask;

		cv::Mat _color_masked;

		cv::Mat _mask1;
		cv::Mat _mask2;
		cv::Mat _mask3;
	
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
	user->_centroid_2d.x = yy;

	kinect->get_user_mask(data->getId(),user->_mask);

	cv::dilate(user->_mask,user->_mask,cv::Mat(3,3,CV_8UC1));
	cv::erode(user->_mask,user->_mask,cv::Mat(7,7,CV_8UC1));

	color.copyTo(user->_color_masked,user->_mask);
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

void rgb_histogram(cv::Mat image, int kinect, int user){
	// Separate the image in 3 places ( B, G and R )
	cv::vector<cv::Mat> bgr_planes;
	cv::split( image, bgr_planes );

	/// Establish the number of bins
	int histSize = 256;

	/// Set the ranges ( for B,G,R) )
	float range[] = { 0, 256 } ;
	const float* histRange = { range };

	bool uniform = true; bool accumulate = false;

	cv::Mat b_hist, g_hist, r_hist;

	/// Compute the histograms:
	cv::calcHist( &bgr_planes[0], 1, 0, cv::Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
	cv::calcHist( &bgr_planes[1], 1, 0, cv::Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
	cv::calcHist( &bgr_planes[2], 1, 0, cv::Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

	// Draw the histograms for B, G and R
	int hist_w = 512; int hist_h = 400;
	int bin_w = cvRound( (double) hist_w/histSize );

	cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

	/// Normalize the result to [ 0, histImage.rows ]
	cv::normalize(b_hist, b_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(g_hist, g_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(r_hist, r_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

	/// Draw for each channel
	for( int i = 1; i < histSize; i++ )
	{
		cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
						cv::Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
						cv::Scalar( 255, 0, 0), 2, 8, 0  );
		cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
						cv::Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
						cv::Scalar( 0, 255, 0), 2, 8, 0  );
		cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
						cv::Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
						cv::Scalar( 0, 0, 255), 2, 8, 0  );
	}

	char win_hist[128];
	char win_user[128];
	sprintf(win_hist,"RGB Hist(%d-%d)",kinect,user);
	sprintf(win_user,"RGB User (%d-%d)",kinect,user);
	cv::imshow(win_hist, histImage );
	cv::imshow(win_user, image );
}

void hsv_histogram(cv::Mat image, cv::Mat mask, int kinect, int user){
	cv::Mat hsv_base, hsv_half_down;;

	cv::cvtColor( image, hsv_base, CV_BGR2HSV );

	hsv_half_down = hsv_base(	cv::Range( hsv_base.rows/2, hsv_base.rows - 1 ), 
								cv::Range( 0, hsv_base.cols - 1 ) );

	/// Using 30 bins for hue and 32 for saturation
	int h_bins = 50; int s_bins = 60;
	int histSize[] = { h_bins, s_bins };

	// hue varies from 0 to 256, saturation from 0 to 180
	float h_ranges[] = { 0, 256 };
	float s_ranges[] = { 0, 180 };

	const float* ranges[] = { h_ranges, s_ranges };

	// Use the o-th and 1-st channels
	int channels[] = { 0, 1 };

	/// Histograms
	cv::MatND hist_base;

	/// Calculate the histograms for the HSV images
	cv::calcHist( &hsv_base, 1, channels, mask, hist_base, 2, histSize, ranges, true, false );
	cv::normalize( hist_base, hist_base, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

	for(int i = 0 ; i < n_images ; ++i){
		cv::MatND hist_rec;

		cv::calcHist( &mat_hsv[i], 1, channels, mat_mask[i], hist_rec, 2, histSize, ranges, true, false );
		cv::normalize( hist_rec, hist_rec, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

		int compare_method = 0;
		double compare_value = compareHist( hist_base, hist_rec, compare_method );

		printf("%.4f\t",compare_value);

		char win_comp[128];
		sprintf(win_comp,"HSV compare (%d)",i);
		cv::imshow(win_comp, mat_hsv_mask[i] );
	}
	printf("\n");

	char win_user[128];
	sprintf(win_user,"HSV User (%d-%d)",kinect,user);
	cv::imshow(win_user, hsv_base );

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

}

int main_multi_tracking(int argc, char* argv[]){
	NIKinect2Manager* kinect_manager = new NIKinect2Manager();

	int n_kinects = kinect_manager->initialize_all_kinects();

	load_images();

	char c = 0;
	while((c = cv::waitKey(500)) != 27){
		if(!kinect_manager->update_all())
			break;

		for(int i = 0 ; i < n_kinects ; ++i){
			cv::Mat color,depth,mask;

			NIKinect2* kinect = kinect_manager->get_kinect(i);

			if(kinect){
				kinect->get_depth_8(depth);
				kinect->get_color(color);
				kinect->get_users_map(mask);

				std::vector<nite::UserData*>* users_data = kinect->get_users_data();
				std::vector<int> *users_ids = kinect->get_users_ids();

				if(users_data && users_ids){
					//printf("Users: ");
					for(int k = 0 ; k < users_ids->size() ; ++k){
						nite::UserData data;

						if(kinect->get_user_data(users_ids->at(k),data) && data.getCenterOfMass().x){
							User user;

							init_user(&user, kinect, &data, color);

							cv::Mat just_user;
							cv::Mat user_mask;
							//user._color_masked(user._bbox).copyTo(just_user);
							color(user._bbox).copyTo(just_user);
							mask(user._bbox).copyTo(user_mask);

							rgb_histogram(just_user,i,k);

							hsv_histogram(just_user,user_mask,i,k);

							if(c == 'c'){
								static int ac = 0;

								char buff_color[128];
								char buff_depth[128];
								sprintf(buff_color,"Track\\Recordings\\%d_color.png",ac);
								sprintf(buff_depth,"Track\\Recordings\\%d_depth.png",ac);
								cv::Mat rec_color,rec_depth;

								color(user._bbox).copyTo(rec_color);
								user._mask(user._bbox).copyTo(rec_depth);

								cv::imwrite(buff_color,rec_color);
								cv::imwrite(buff_depth,rec_depth);
								ac++;

							}
							//hsv_histogram(just_user,i,k);

							//split_user(user);
							//
							//char win_users[128];
							//char win_users_mask1[128];
							//char win_users_mask2[128];
							//sprintf(win_users,"Users(%d) ID(%d)",i,data.getId());
							//sprintf(win_users_mask1,"Users(%d) ID(%d) Mask1",i,data.getId());
							//sprintf(win_users_mask2,"Users(%d) ID(%d) Mask2",i,data.getId());
							//cv::imshow(win_users,user->_color_masked);
							//cv::imshow(win_users_mask1,user->_mask1);
							//cv::imshow(win_users_mask2,user->_mask2);
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
	}
	kinect_manager->~NIKinect2Manager();

	return 0;
}