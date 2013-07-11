#include "..\ThreadedGUI\main_gui.h"
#include <boost\thread\thread.hpp>
#include <boost\thread\condition_variable.hpp>
#include <Windows.h>
#include <NIThreadedKinect.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "..\ThreadedPCL\Viewer3D.h"

Viewer3D *viewer3d;

bool new_cloud = true;
boost::mutex mutex_viewer;
boost::condition_variable condition_viewer;
boost::mutex mutex_consumer;
boost::condition_variable condition_consumer;

bool color_flag = false;

double start_t;
double end_t;

void myfunc_pcl_viewer(NIThreadedKinect* kinect, pcl::visualization::CloudViewer* viewer,pcl::PointCloud<pcl::PointXYZ> *cloud, pcl::PointCloud<pcl::PointXYZRGB> *cloud_color, bool *running){
	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

	while(*running){
		{
			boost::mutex::scoped_lock lock(mutex_viewer);
			//start_t = cv::getTickCount();
			condition_viewer.wait(lock);
		}

		if(color_flag)
			viewer3d->viewer->showCloud(viewer3d->cloud_color.makeShared());
		else
			viewer3d->viewer->showCloud(viewer3d->cloud.makeShared());

		++_frame_counter;
		if (_frame_counter == 15)
		{
			double current_tick = cv::getTickCount();
			_frame_rate = _frame_counter / ((current_tick - _last_tick)/cv::getTickFrequency());
			_last_tick = current_tick;
			_frame_counter = 0;
			printf("\t\t PCL FrameRate %.2f\n",_frame_rate);
		}
	}
}

void myfunc_pcl_consumer(NIThreadedKinect* kinect,pcl::PointCloud<pcl::PointXYZ> *cloud, pcl::PointCloud<pcl::PointXYZRGB> *cloud_color, bool *running){
	while(*running){

		kinect->mutex_lock(NIThreadedKinect::POINT_CLOUD_T);
			if(color_flag){
				if(kinect->consume_copy_point_cloud(viewer3d->cloud_color)){
					//New Cloud
					condition_viewer.notify_all();
				}
			}
			else{
				if(kinect->consume_copy_point_cloud(viewer3d->cloud)){
					//New Cloud
					condition_viewer.notify_all();
				}
			}
		kinect->mutex_unlock(NIThreadedKinect::POINT_CLOUD_T);
		{
			boost::mutex::scoped_lock lock(mutex_consumer);
			condition_consumer.wait(lock);
		}
	}
}

void func_pcl_viewer(NIThreadedKinect* kinect, pcl::visualization::CloudViewer* viewer,pcl::PointCloud<pcl::PointXYZ> *cloud, pcl::PointCloud<pcl::PointXYZRGB> *cloud_color, bool *running){
	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;


	while(*running){
		//kinect->mutex_lock(NIThreadedKinect::POINT_CLOUD_T);
		//	if(kinect->consume_copy_point_cloud(*cloud)){
		//		//New Cloud
		//		viewer->showCloud(cloud->makeShared());
		//		//condition_viewer.notify_all();
		//	}
		//kinect->mutex_unlock(NIThreadedKinect::POINT_CLOUD_T);
		//viewer->showCloud(cloud->makeShared());
		
		{
			boost::mutex::scoped_lock lock(mutex_viewer);
			//start_t = cv::getTickCount();
			condition_viewer.wait(lock);
		}

		if(color_flag)
			viewer->showCloud(cloud_color->makeShared());
		else
			viewer->showCloud(cloud->makeShared());

		++_frame_counter;
		if (_frame_counter == 15)
		{
			double current_tick = cv::getTickCount();
			_frame_rate = _frame_counter / ((current_tick - _last_tick)/cv::getTickFrequency());
			_last_tick = current_tick;
			_frame_counter = 0;
			printf("\t\t PCL FrameRate %.2f\n",_frame_rate);
		}
	}
}

void func_pcl_consumer(NIThreadedKinect* kinect,pcl::PointCloud<pcl::PointXYZ> *cloud, pcl::PointCloud<pcl::PointXYZRGB> *cloud_color, bool *running){
	while(*running){

		kinect->mutex_lock(NIThreadedKinect::POINT_CLOUD_T);
			if(color_flag){
				if(kinect->consume_copy_point_cloud(*cloud_color)){
					//New Cloud
					condition_viewer.notify_all();
				}
			}
			else{
				if(kinect->consume_copy_point_cloud(*cloud)){
					//New Cloud
					condition_viewer.notify_all();
				}
			}
		kinect->mutex_unlock(NIThreadedKinect::POINT_CLOUD_T);
		{
			boost::mutex::scoped_lock lock(mutex_consumer);
			condition_consumer.wait(lock);
		}
	}
}

int main(int argc, char* argv[]){
	//boost::thread gui_thread(main_gui,argc,argv);

	MainGUI *gui = new MainGUI(argc,argv);
	
	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

	NIThreadedKinect* kinect = new NIThreadedKinect();
	
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width = 640*480;
	cloud.height = 1;
	cloud.points.resize (cloud.width * cloud.height);

	pcl::PointCloud<pcl::PointXYZRGB> cloud_color;
	cloud_color.width = 640*480;
	cloud_color.height = 1;
	cloud_color.points.resize (cloud.width * cloud.height);

	pcl::visualization::CloudViewer viewer("Simple cloud_file Viewer");

	viewer3d = new Viewer3D(kinect);

	cv::Mat depth;
	cv::Mat depth8;
	cv::Mat color;

	kinect->init("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni");

	bool running = true;
	char ch = 0;
	//cv::namedWindow("Color");

	kinect->set_3d_analysis_step(2);
	kinect->set_processing_flag(NIKinect::POINT_CLOUD_PCL,true);
	//Init Kinect
	kinect->start_thread(NIThreadedKinect::CAPTURE_T);
	Sleep(100);
	kinect->start_thread(NIThreadedKinect::POINT_CLOUD_T);
	Sleep(100);

	//NIKinect* kinect = new NIKinect();
	//kinect->set_processing_flag(NIKinect::DEPTH_COLOR, true);

	//kinect->init("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni");

	//boost::thread kinect_thread(&NIKinect::run, kinect);

	//gui->add_kinect(kinect);
	
	boost::thread gui_thread(&MainGUI::run, gui,argc,argv);
	Sleep(200);

	
	boost::thread t_pcl(&myfunc_pcl_consumer,kinect,&cloud,&cloud_color,&running);
	Sleep(100);
	boost::thread t_viewer(&myfunc_pcl_viewer,kinect,&viewer,&cloud,&cloud_color,&running);

	cv::Mat3b mat1;
	cv::Mat1f mat2;

	while(running){
		//Update Images
		kinect->mutex_lock(NIThreadedKinect::CAPTURE_T);
			kinect->get_color(color);
			kinect->get_depth(depth);
		kinect->mutex_unlock(NIThreadedKinect::CAPTURE_T);

		if(kinect->new_point_cloud()){
			condition_consumer.notify_all();
		}

		//if(color_flag != gui->viewer->flag_color){
		//	kinect->set_processing_flag(NIKinect::POINT_CLOUD_PCL_COLOR,gui->viewer->flag_color);
		//	kinect->set_processing_flag(NIKinect::POINT_CLOUD_PCL,!gui->viewer->flag_color);
		//	color_flag = gui->viewer->flag_color;
		//}
		//Update Point Cloud
		//kinect->mutex_lock(NIThreadedKinect::POINT_CLOUD_T);
		//	if((cloud_ptr = kinect->get_point_cloud())){
		//		viewer.showCloud(cloud_ptr->makeShared());
		//	}
		//kinect->mutex_unlock(NIThreadedKinect::POINT_CLOUD_T);

		//Show Images
		depth.convertTo(depth8,CV_8UC1,0.05);
		//
		//cv::imshow("Depth", depth8);

		color.copyTo(mat1);
		depth8.copyTo(mat2);

		//cv::imshow("Color", mat1);

		gui->viewer->_ntk_widget_right->setImage(mat1);
		gui->viewer->_ntk_widget_left->setImage(mat2);


		//Update and Keys 
		ch = cv::waitKey(21);
		if(ch == 27) 
			running = false;
		
		if(ch == 'c') 
			color_flag = !color_flag;

		_frame_counter++;
		if (_frame_counter == 15)
		{
			double current_tick = cv::getTickCount();
			_frame_rate = _frame_counter / ((current_tick - _last_tick)/cv::getTickFrequency());
			_last_tick = current_tick;
			_frame_counter = 0;
			printf("FrameRate %.2f\n",_frame_rate);
		}
	}


	//cv::circle(dc,cv::Point(300,300),10,cv::Scalar(0,0,255),-1);

	//while(true){
	//	printf("Main sleep\n");
	//	//kinect->get_depth_as_color(dc);

	//	gui->viewer->_ntk_widget_right->setImage(dc);

	//	//cv::imshow("image",dc);
	//	//cv::waitKey(100);
	//	Sleep(1009);
	//	//gui->update();
	//}

	gui_thread.join();

	printf("End\n");
	Sleep(10000);

	return 0;
}