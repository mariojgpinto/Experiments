#include "NIViewer.h"

#include <NIKinect2Manager.h>

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

int main_pcl_ni_kinect(int argc, char* argv[]){
	NIKinect2Manager* kinect_manager = new NIKinect2Manager();

	int n_kinects = kinect_manager->initialize_all_kinects();

	pcl::visualization::CloudViewer viewer("Simple cloud_file Viewer");
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	cloud.width = 640*480;
	cloud.height = 1;
	cloud.points.resize (cloud.width * cloud.height);

	char c = 0;
	while((c = cv::waitKey(11)) != 27){
		if(!kinect_manager->update_all())
			break;

		//Reset Points
		cloud.points.clear();

		for(int i = 0 ; i < n_kinects ; ++i){
			cv::Mat color,depth,mask,user;

			NIKinect2* kinect = kinect_manager->get_kinect(i);

			if(kinect){
				//kinect->get_depth_8(depth);
				//kinect->get_color(color);
		
				//char win_color[128];
				//char win_depth[128];
				//sprintf(win_color,"Color(%d)",i);
				//sprintf(win_depth,"Depth(%d)",i);
				//cv::imshow(win_color,color);
				//cv::imshow(win_depth,depth);

				//****************************************************************
				//* 3D
				//****************************************************************
				
				uint16_t* ptr = (uint16_t*)kinect->get_depth_frame_ref()->getData();
				openni::RGB888Pixel* ptr_clr = (openni::RGB888Pixel*)kinect->get_color_frame_ref()->getData();
				openni::VideoStream *depthStream = kinect->get_depth_stream();
				for(int y=0; y<480; y++) { 
					for(int x=0; x<640; x++) {
						float d_x = x;
						float d_y = y;
						float d_z =  ptr[y * 640 + x];
						float w_x = 0;
						float w_y = 0;
						float w_z = 0;

						if(d_z > 0){
							openni::CoordinateConverter::convertDepthToWorld(*depthStream,d_x,d_y,d_z,&w_x,&w_y,&w_z);

							pcl::PointXYZRGB pt(ptr_clr[y * 640 + x].r,
												ptr_clr[y * 640 + x].g,
												ptr_clr[y * 640 + x].b);
							pt.x = w_x;
							pt.y = w_y;
							pt.z = w_z;

							cloud.push_back(pcl::PointXYZRGB(pt));
						}
					}
				} 


			} //Kinect
		} //N Kinects

		viewer.showCloud(cloud.makeShared());
	} //While
	kinect_manager->~NIKinect2Manager();

	return 0;
}