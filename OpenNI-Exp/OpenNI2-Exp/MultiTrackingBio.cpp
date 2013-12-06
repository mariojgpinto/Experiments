#include "MultiTrackingBio.h"

#include <NIKinect2Manager.h>

namespace{

#define M_PI 3.141592654

class User{
	public:
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
		
		cv::Mat _color_user;
		cv::Mat _mask_user;
		
		int _id;
		double _conf;
};

void init_user(User* user, NIKinect2* kinect, nite::UserData* data, cv::Mat color){
	if(!user || !kinect || !data || !color.rows || !color.cols) return;

	color.copyTo(user->_color_full);

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

	kinect->get_user_mask(data->getId(),user->_mask_full);

	cv::dilate(user->_mask_full,user->_mask_full,cv::Mat(3,3,CV_8UC1));
	cv::erode(user->_mask_full,user->_mask_full,cv::Mat(7,7,CV_8UC1));

	color(user->_bbox).copyTo(user->_color_user);
	user->_mask_full(user->_bbox).copyTo(user->_mask_user);
}

}

int main_multi_tracking_bio(int argc, char* argv[]){
	NIKinect2Manager* kinect_manager = new NIKinect2Manager();

	NIKinect2::ni_initialize();

	int n_kinects = kinect_manager->initialize_all_kinects();

	std::vector<std::vector<User*>> users_kinect(n_kinects);
	int* users_kinect_ac = (int*)malloc(sizeof(int) * n_kinects);

	for(int k = 0 ; k < n_kinects ; ++k){
		for(int i = 0 ; i < 16 ; ++i){
			users_kinect[k].push_back(new User());
		}
		users_kinect_ac[k] = 0;
	}

	char c = 0;
	while((c = cv::waitKey(11)) != 27){
		if(!kinect_manager->update_all())
			break;

		for(int k = 0 ; k < n_kinects ; ++k){
			cv::Mat color,depth,mask;

			NIKinect2* kinect = kinect_manager->get_kinect(k);
			
			if(kinect){
				kinect->get_depth_8(depth);
				kinect->get_color(color);
				kinect->get_users_map(mask);

				std::vector<nite::UserData*>* users_data = kinect->get_users_data();
				std::vector<int> *users_ids = kinect->get_users_ids();

				if(users_data && users_ids){
					users_kinect_ac[k] = 0;
					for(int u = 0 ; u < users_ids->size() ; ++u){
						nite::UserData data;

						if(kinect->get_user_data(users_ids->at(u),data) && data.getCenterOfMass().x){
							init_user(users_kinect[k][u], kinect, &data, color);

							users_kinect_ac[k]++;

							cv::circle(color,users_kinect[k][u]->_centroid_2d,7,cv::Scalar(0,0,255),-1);
							cv::rectangle(color,users_kinect[k][u]->_bbox,cv::Scalar(0,255,0),3);
						}
					}

					if(users_ids->size() == 2){

					}
				}

				char win_color[128];
				sprintf(win_color,"Color(%d)",k);
				cv::imshow(win_color,color);
			}
		}
	} //while

	kinect_manager->~NIKinect2Manager();

	return 0;
}