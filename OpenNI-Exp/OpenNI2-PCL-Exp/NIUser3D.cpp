#include "NIUser3D.h"

#include "NIViewer.h"

#include <NIKinect2Manager.h>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace{

class User{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		User(){
			_cloud.width = 640*480;
			_cloud.height = 1;
			_cloud.points.resize (_cloud.width * _cloud.height);
		}
		User(User* user){
			_cloud.width = 640*480;
			_cloud.height = 1;
			_cloud.points.resize (_cloud.width * _cloud.height);

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
		

		pcl::PointCloud<pcl::PointXYZRGB> _cloud;

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

	kinect->get_user_mask(data->getId(),user->_mask_full);

	cv::dilate(user->_mask_full,user->_mask_full,cv::Mat(3,3,CV_8UC1));
	cv::erode(user->_mask_full,user->_mask_full,cv::Mat(7,7,CV_8UC1));

	color(user->_bbox).copyTo(user->_color_user);
	user->_mask_full(user->_bbox).copyTo(user->_mask_user);
}

void generate_3d(User* user){
	if(!user) return;
	
	uint16_t* ptr = (uint16_t*)user->_kinect->get_depth_frame_ref()->getData();
	openni::RGB888Pixel* ptr_clr = (openni::RGB888Pixel*)user->_kinect->get_color_frame_ref()->getData();
	openni::VideoStream *depthStream = user->_kinect->get_depth_stream();
			
	uchar* mask_ptr = (uchar*)user->_mask_full.data;

	user->_cloud.clear();

	int idx = 0;
	for(int y = user->_bbox.y ; y < user->_bbox.y + user->_bbox.height ; ++y){
		for(int x = user->_bbox.x ; x < user->_bbox.x + user->_bbox.width ; ++x){
			idx =y * 640 + x;
			if(mask_ptr[idx]){
				float d_x = x;
				float d_y = y;
				float d_z =  ptr[idx];
				float w_x = 0;
				float w_y = 0;
				float w_z = 0;

				if(d_z > 0){
					openni::CoordinateConverter::convertDepthToWorld(*depthStream,d_x,d_y,d_z,&w_x,&w_y,&w_z);

					pcl::PointXYZRGB pt(ptr_clr[idx].r,
										ptr_clr[idx].g,
										ptr_clr[idx].b);
					pt.x = w_x;
					pt.y = w_y;
					pt.z = w_z;

					user->_cloud.push_back(pcl::PointXYZRGB(pt));
				}
			}
		}
	}
}

}

int main_pcl_ni_user_3d(int argc, char* argv[]){
	NIKinect2Manager* kinect_manager = new NIKinect2Manager();

	int n_kinects = kinect_manager->initialize_all_kinects();

	std::vector<std::vector<User*>> users_kinect(n_kinects);
	int* users_kinect_ac = (int*)malloc(sizeof(int) * n_kinects);

	for(int k = 0 ; k < n_kinects ; ++k){
		for(int i = 0 ; i < 16 ; ++i){
			users_kinect[k].push_back(new User());
		}
		users_kinect_ac[k] = 0;
	}

	pcl::visualization::PCLVisualizer viewer("Simple cloud_file Viewer");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
	viewer.addCoordinateSystem (1.0);
	viewer.initCameraParameters ();

	char c = 0;
	while((c = cv::waitKey(11)) != 27){
		if(!kinect_manager->update_all())
			break;

		

		for(int k = 0 ; k < n_kinects ; ++k){
			cv::Mat color,depth,mask,user;

			NIKinect2* kinect = kinect_manager->get_kinect(k);

			if(kinect){
				kinect->get_depth_8(depth);
				kinect->get_color(color);
		


				std::vector<nite::UserData*>* users_data = kinect->get_users_data();
				std::vector<int> *users_ids = kinect->get_users_ids();

				if(users_data && users_ids){
					users_kinect_ac[k] = 0;
					for(int u = 0 ; u < users_ids->size() ; ++u){
						nite::UserData data;

						if(kinect->get_user_data(users_ids->at(u),data) && data.getCenterOfMass().x){
							init_user(users_kinect[k][u], kinect, &data, color);

							users_kinect_ac[k]++;

							generate_3d(users_kinect[k][u]);

							//cv::circle(color,users_kinect[k][u]->_centroid_2d,7,cv::Scalar(0,0,255),-1);
							//cv::rectangle(color,users_kinect[k][u]->_bbox,cv::Scalar(0,255,0),3);


							char win_color_user[128];
							char win_depth_user[128];
							sprintf(win_color_user,"Color(%d)(%d)",k,u);
							sprintf(win_depth_user,"Depth(%d)(%d)",k,u);
							cv::imshow(win_color_user,users_kinect[k][u]->_color_user);
							cv::Mat temp_user;
							users_kinect[k][u]->_color_full.copyTo(temp_user,users_kinect[k][u]->_mask_full);
							cv::circle(temp_user,users_kinect[k][u]->_centroid_2d,7,cv::Scalar(0,0,255),-1);
							cv::imshow(win_depth_user,temp_user);
						}
					}
				}
			} //Kinect
		} //N Kinects

		std::vector<User*> temp_users;
		for(int i = 0 ; i < users_kinect_ac[0] ; ++i){
			temp_users.push_back(users_kinect[0][i]);
		}
		for(int i = 0 ; i < users_kinect_ac[1] ; ++i){
			temp_users.push_back(users_kinect[1][i]);
		}

		//Reset Points
		if(temp_users.size()){
			//cloud.points.clear();
			viewer.removeAllPointClouds();

			for(int u = 0 ; u < temp_users.size() ; ++u){
				char cloud_name[128];
				sprintf(cloud_name,"Cloud%d",u);

				viewer.addPointCloud(temp_users[u]->_cloud.makeShared(),cloud_name);
			}
			
			viewer.spinOnce(50);
		}
	} //While
	kinect_manager->~NIKinect2Manager();

	return 0;
}