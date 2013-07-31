#include <_ID.h>

#include "pcl_nikinect.h"

#include <NIKinect.h>
#include <ToolBoxPCL.h>

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>


int main_pcl_ni_kinect(int argc, char* argv[]){
	bool result = false;
	NIKinect* kinect = NULL;

	cv::Mat color;
	cv::Mat depth;
	cv::Mat mask;
	cv::Mat depthMat8UC1;
	cv::Mat depthMat16UC1;
	cv::Mat depth_as_color;

	kinect = new NIKinect();


	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

#ifdef _CCG
	result = kinect->init(NULL,1+NIKinect::SCENE_A);//"C:\\Dev\\RDCC\\Project\\Data\\movement_1.oni");
#endif

#ifdef _HOME
	result = kinect->init("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni",NIKinect::DEPTH_G + NIKinect::IMAGE_G); 
#endif

	//kinect->set_processing_flag(NIKinect::DEPTH_COLOR, true);
	kinect->set_processing_flag(NIKinect::POINT_CLOUD, true);

	xn::DepthGenerator _depth_generator = *kinect->get_depth_generator();	

	XnPoint3D * pointList = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 
	XnPoint3D * realWorld;// = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 
	xn::DepthMetaData _depth_md;

	pcl::visualization::CloudViewer viewer("Simple cloud_file Viewer");
	//pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width = 640*480;
	cloud.height = 1;
	cloud.points.resize (cloud.width * cloud.height);
	int point_step = 1; kinect->set_3d_analysis_step(point_step);
	int n = (XN_VGA_Y_RES*XN_VGA_X_RES) / (point_step*point_step);
	char ch;
	int counter = 0;
	while((ch = cv::waitKey(31)) != 27){
		if(!kinect->update()) 
			break;
		
		if(ch == ' '){
			point_step++;
			n = (XN_VGA_Y_RES*XN_VGA_X_RES) / (point_step*point_step);
			kinect->set_3d_analysis_step(point_step);
		}
		result = kinect->get_depth(depthMat16UC1);
		//result = kinect->get_color(color);
		//result = kinect->get_depth_meta_data(_depth_md);

		realWorld = kinect->get_points_3d();
		//int ac = 0;
		//for(int y=0; y<XN_VGA_Y_RES; y+=point_step) { 
		//	for(int x=0; x<XN_VGA_X_RES; x+=point_step) { 
		//		//if(! (x % 2)){
		//			XnPoint3D point1;
		//			point1.X = x; 
		//			point1.Y = y; 
		//			point1.Z = _depth_md[y * XN_VGA_X_RES + x]; 

		//			//pointList[y * XN_VGA_X_RES + x] = point1;
		//			pointList[ac] = point1;
		//			ac++;
		//		//}
		//	}
		//} 
		//printf("%d\n",ac);

		//_depth_generator.ConvertProjectiveToRealWorld(n, pointList, realWorld);

		if(pointList){
			
			ToolBoxPCL::convert_points_to_mesh(n, realWorld,cloud);

			viewer.showCloud(cloud.makeShared());
		}


		//cv::imshow("color",color);

		++_frame_counter;
		if (_frame_counter == 10)
		{
			double current_tick = cv::getTickCount();
			_frame_rate = _frame_counter / ((current_tick - _last_tick)/cv::getTickFrequency());
			_last_tick = current_tick;
			_frame_counter = 0;
			printf("%.2f\t\t%.2f\n",_frame_rate, kinect->get_frame_rate());
		}
	}

	return 0;
}