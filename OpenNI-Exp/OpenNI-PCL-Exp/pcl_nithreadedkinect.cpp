#include "pcl_nithreadedkinect.h"

#include <_ID.h>
#include <NIThreadedKinect.h>

#include <XnCppWrapper.h>
using namespace xn;

#include <opencv2\opencv.hpp>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <ToolBoxPCL.h>

int main_nithreadedkinect(int argc, char *argv[]){
	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

	NIThreadedKinect* kinect = new NIThreadedKinect();

	cv::Mat depth;
	cv::Mat depth8;
	cv::Mat color;

	XnPoint3D * realWorld = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 
	pcl::visualization::CloudViewer viewer("Simple cloud_file Viewer");
	pcl::PointCloud<pcl::PointXYZ> cloud;

	cv::namedWindow("Depth");
	cv::namedWindow("Color");

#ifdef _CCG
	kinect->init();
#endif

#ifdef _HOME
	kinect->init("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers2.oni");
#endif
	bool running = true;
	char ch = 0;

	kinect->set_3d_analysis_step(1);


	kinect->start_thread(NIThreadedKinect::CAPTURE_T);
	Sleep(1000);
	kinect->start_thread(NIThreadedKinect::POINT_CLOUD_T);
	Sleep(100);
	//pcl::visualization::CloudViewer viewer("Simple cloud_file Viewer");
	//pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width = 640*480;
	cloud.height = 1;
	cloud.points.resize (cloud.width * cloud.height);
	
	

	while(running){
		kinect->mutex_lock(NIThreadedKinect::CAPTURE_T);
		
		kinect->get_color(color);
		kinect->get_depth(depth);

		if(kinect->get_3d_copy(realWorld)){
			cloud.points.clear();

			//for(int y=0; y<XN_VGA_Y_RES; y++) { 
			//	for(int x=0; x<XN_VGA_X_RES; x++) { 
			//		if(realWorld[y * XN_VGA_X_RES + x].Z > 0.0){
			//			//pcl::PointXYZRGB pt(ptr_clr[y * XN_VGA_X_RES * 3 + x* 3 + 2],
			//			//					ptr_clr[y * XN_VGA_X_RES * 3 + x* 3 + 1],
			//			//					ptr_clr[y * XN_VGA_X_RES * 3 + x* 3 + 0]);
			//			//pt.x = realWorld[y * XN_VGA_X_RES + x].X;
			//			//pt.y = -realWorld[y * XN_VGA_X_RES + x].Y;
			//			//pt.z = realWorld[y * XN_VGA_X_RES + x].Z;
			//			//
			//			//cloud.points[y * XN_VGA_X_RES + x] = pt;
			//			//cloud.push_back(pt);
			//			cloud.push_back(pcl::PointXYZ(realWorld[y * XN_VGA_X_RES + x].X,-realWorld[y * XN_VGA_X_RES + x].Y,realWorld[y * XN_VGA_X_RES + x].Z));
			//	//		ac++;
			//		}
			//	} 
			//}
			ToolBoxPCL::convert_points_to_mesh(XN_VGA_Y_RES*XN_VGA_X_RES,realWorld,cloud);

			printf("New Cloud\n");
		}

		kinect->mutex_unlock(NIThreadedKinect::CAPTURE_T);

		depth.convertTo(depth8,CV_8UC1,0.05);
		cv::imshow("Color", color);
		cv::imshow("Depth", depth8);
		ch = cv::waitKey(12);
		viewer.showCloud(cloud.makeShared());

		
		//uchar* depth_ptr = depthMat16UC1.data;
		if(ch == 27) 
			running = false;
		
		++_frame_counter;
		if (_frame_counter == 15)
		{
			double current_tick = cv::getTickCount();
			_frame_rate = _frame_counter / ((current_tick - _last_tick)/cv::getTickFrequency());
			_last_tick = current_tick;
			_frame_counter = 0;
			printf("%.2f\n",_frame_rate);
		}
	}

	return 0;
}