#include "MultiNIKinect2.h"

#include <NIKinect2Manager.h>


int main_multi_nikinect2(int argc, char* argv[]){
	NIKinect2Manager* kinect_manager = new NIKinect2Manager();

	int n_kinects = kinect_manager->initialize_all_kinects();

	char c = 0;
	while((c = cv::waitKey(11)) != 27){
		if(!kinect_manager->update_all())
			break;

		for(int i = 0 ; i < n_kinects ; ++i){
			cv::Mat color,depth,mask,user;

			NIKinect2* kinect = kinect_manager->get_kinect(i);

			if(kinect){
				kinect->get_depth_8(depth);
				kinect->get_color(color);
				kinect->get_users_map(mask);
				color.copyTo(user,mask);

				std::vector<nite::UserData*>* users_data = kinect->get_users_data();
				std::vector<int> *users_ids = kinect->get_users_ids();

				if(users_data && users_ids){
					//printf("Users: ");
					for(int k = 0 ; k < users_ids->size() ; ++k){
						nite::UserData data;

						if(kinect->get_user_data(users_ids->at(k),data)){
							const nite::BoundingBox bbox = data.getBoundingBox();

							cv::rectangle(color,cv::Rect(bbox.min.x,bbox.min.y,bbox.max.x - bbox.min.x, bbox.max.y - bbox.min.y),
										  cv::Scalar(255,0,0),3);

							cv::Mat umask,user;
							kinect->get_user_mask(users_ids->at(k),umask);
							color.copyTo(user,umask);

							//char win_users[128];
							//sprintf(win_users,"Users(%d) ID(%d)",i,data.getId());
							//cv::imshow(win_users,user);
						}
					}
				}

				char win_color[128];
				char win_depth[128];
				
				sprintf(win_color,"Color(%d)",i);
				sprintf(win_depth,"Depth(%d)",i);
				
				cv::imshow(win_color,color);
				cv::imshow(win_depth,depth);

				//char win_users[128];
				//sprintf(win_users,"Users(%d)",i);
				//cv::imshow(win_users,user);
			}
		}
	}
	kinect_manager->~NIKinect2Manager();

	return 0;
}

//int main_multi_nikinect2(int argc, char* argv[]){
//	NIKinect2::ni_initialize();
//
//	NIKinect2* kinect = new NIKinect2();
//
//	kinect->initialize();
//
//	kinect->set_depth_color_registration(true);
//
//	kinect->enable_depth_generator();
//	kinect->enable_color_generator();
//	kinect->enable_user_generator();
//
//	cv::Mat image_depth, image_color;
//	cv::Mat mask_user;
//
//	char c = 0;
//	while((c = cv::waitKey(11)) != 27){
//		NIKinect2::ni_update();
//
//		kinect->update();
//
//		kinect->get_depth_8(image_depth);
//		kinect->get_color(image_color);
//		kinect->get_users_map(mask_user);
//
//		cv::Mat image_user;image_color.copyTo(image_user,mask_user);
//
//		cv::imshow("Depth",image_depth);
//		cv::imshow("Color",image_color);
//		cv::imshow("User",image_user);
//
//		if(c == 'r'){
//			kinect->set_depth_color_registration(true);
//		}
//	}
//
//	return 0;
//}