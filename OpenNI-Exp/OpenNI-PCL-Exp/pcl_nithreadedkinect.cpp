#include <_ID.h>
#include <XnCppWrapper.h>
using namespace xn;

#include <opencv2\opencv.hpp>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <NIThreadedKinect.h>

#include <boost\thread\thread.hpp>

#include <ToolBoxPCL.h>

bool new_cloud = true;


void func_pcl(NIThreadedKinect* kinect, pcl::visualization::CloudViewer* viewer, bool *running){
	pcl::PointCloud<pcl::PointXYZ>* cloud_ptr;
	
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width = 640*480;
	cloud.height = 1;
	cloud.points.resize (cloud.width * cloud.height);

	while(*running){
		if(kinect->mutex_try_lock(NIThreadedKinect::POINT_CLOUD_T)){
			if(kinect->copy_point_cloud(cloud)){
				//FILE* fp = fopen("points.txt","w+");
				//for(int i = 0 ; i < cloud.size() ; i++){
				//	fprintf(fp,"%.4f , %.4f , %.4f\n",cloud[i].x,cloud[i].y,cloud[i].z);
				//}
				//fclose(fp);
				
				//New Cloud
				viewer->showCloud(cloud.makeShared());
			}
			kinect->mutex_unlock(NIThreadedKinect::POINT_CLOUD_T);
		}
	}
}

void func_kinect(NIThreadedKinect* kinect, bool running){
	cv::Mat depth;
	cv::Mat depth8;
	cv::Mat color;

	while(running){
		if(kinect->mutex_try_lock(NIThreadedKinect::CAPTURE_T)){
			kinect->get_color(color);
			kinect->get_depth(depth);
			kinect->mutex_unlock(NIThreadedKinect::CAPTURE_T);

			depth.convertTo(depth8,CV_8UC1,0.05);
			cv::imshow("Color", color);
			cv::imshow("Depth", depth8);
		}
	}
}

int main_nithreadedkinect(int argc, char* argv[]){
	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

	NIThreadedKinect* kinect = new NIThreadedKinect();
	
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::PointXYZ>* cloud_ptr;// = new pcl::PointCloud<pcl::PointXYZ>();

	cv::Mat depth;
	cv::Mat depth8;
	cv::Mat color;

	//Windows
	//pcl::visualization::PCLVisualizer viewer("Simple cloud_file Viewer");
	pcl::visualization::CloudViewer viewer("Simple cloud_file Viewer");
	
	cv::namedWindow("Color");
	cv::namedWindow("Depth");

#ifdef _CCG
	kinect->init();
#endif

#ifdef _HOME
	kinect->init("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers2.oni");
#endif
	bool running = true;
	char ch = 0;

	kinect->set_3d_analysis_step(1);

	//cloud.width = 640*480;
	//cloud.height = 1;
	//cloud.points.resize (cloud.width * cloud.height);

	//Init Kinect
	kinect->start_thread(NIThreadedKinect::CAPTURE_T);
	Sleep(100);
	kinect->start_thread(NIThreadedKinect::POINT_CLOUD_T);
	Sleep(100);
	//viewer.runOnVisualizationThreadOnce(viewerOneOff);
	//viewer.runOnVisualizationThread (viewerPsycho);

	boost::thread t(&func_pcl,kinect,&viewer,&running);

	while(running){
		//Update Images
		kinect->mutex_lock(NIThreadedKinect::CAPTURE_T);
			kinect->get_color(color);
			kinect->get_depth(depth);
		kinect->mutex_unlock(NIThreadedKinect::CAPTURE_T);

		//Update Point Cloud
		//kinect->mutex_lock(NIThreadedKinect::POINT_CLOUD_T);
		//	if((cloud_ptr = kinect->get_point_cloud())){
		//		viewer.showCloud(cloud_ptr->makeShared());
		//	}
		//kinect->mutex_unlock(NIThreadedKinect::POINT_CLOUD_T);

		//Show Images
		depth.convertTo(depth8,CV_8UC1,0.05);
		cv::imshow("Color", color);
		cv::imshow("Depth", depth8);

		//Update and Keys 
		ch = cv::waitKey(21);
		if(ch == 27) 
			running = false;
		
		if(ch == ' ') 
			new_cloud = true;

		++_frame_counter;
		if (_frame_counter == 15)
		{
			double current_tick = cv::getTickCount();
			_frame_rate = _frame_counter / ((current_tick - _last_tick)/cv::getTickFrequency());
			_last_tick = current_tick;
			_frame_counter = 0;
			printf("FrameRate %.2f\n",_frame_rate);
		}
	}

	t.join();

	return 0;
}