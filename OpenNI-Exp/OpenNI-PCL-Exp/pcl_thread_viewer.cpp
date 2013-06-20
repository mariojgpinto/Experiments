#include <_ID.h>

#include "pcl_visualizer.h"

#include <XnOS.h>
#include <math.h>

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

#include <boost\thread.hpp>
#include <exception>

#define SAMPLE_XML_PATH "C:\\Dev\\External\\OpenNI\\Data\\SamplesConfig.xml"

void compute_color_encoded_depth(const cv::Mat1f& depth_im, cv::Mat3b& color_depth_im,
                                     double* i_min_val, double* i_max_val);
float distanceToPlane(const XnPoint3D& p, float a, float b, float c, float d);

XnPoint3D * pointList = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 
XnPoint3D * realWorld = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 
pcl::visualization::CloudViewer viewer("Simple cloud_file Viewer");
pcl::PointCloud<pcl::PointXYZRGB> cloud;
	DepthMetaData _depthMD_copy;
	Context _context;
	ScriptNode _scriptNode;
	DepthGenerator _depth;
	ImageGenerator _image;
	DepthMetaData _depthMD;
	ImageMetaData _imageMD;
	SceneMetaData _sceneMD;
	cv::Mat depthMat8UC1;
	cv::Mat color2;
	cv::Mat color2_pcl;
int yres = XN_VGA_Y_RES - 50;
int xres = XN_VGA_X_RES - 100;
int ac = 0;

boost::mutex mtx_;
boost::mutex mtx_2;

bool running = true;


void pcl_thread_function(){
	Sleep(100);

	while(running){
		//Copy new images
		mtx_.lock();
	
		_depthMD_copy.CopyFrom(_depthMD);
		color2.copyTo(color2_pcl);

		mtx_.unlock();

		//Create buffer
		for(int y=0; y<yres; y++) { 
			for(int x=0; x<xres; x++) { 
				XnPoint3D point1;
				point1.X = x; 
				point1.Y = y; 
				point1.Z = _depthMD_copy[y * XN_VGA_X_RES + x]; 

				pointList[y * XN_VGA_X_RES + x] = point1;
			}
		} 
		//Convert points		
		_depth.ConvertProjectiveToRealWorld(XN_VGA_Y_RES*XN_VGA_X_RES, pointList, realWorld); 

		//Add points to Point Cloud
		//mtx_2.lock();
		cloud.points.clear();
		uint8_t* ptr_clr = (uint8_t*)color2_pcl.data;
		for(int y=0; y<XN_VGA_Y_RES; y++) { 
			for(int x=0; x<XN_VGA_X_RES; x++) { 
				if(realWorld[y * XN_VGA_X_RES + x].Z > 0.0){
					pcl::PointXYZRGB pt(ptr_clr[y * XN_VGA_X_RES * 3 + x* 3 + 2],
										ptr_clr[y * XN_VGA_X_RES * 3 + x* 3 + 1],
										ptr_clr[y * XN_VGA_X_RES * 3 + x* 3 + 0]);
					pt.x = realWorld[y * XN_VGA_X_RES + x].X;
					pt.y = -realWorld[y * XN_VGA_X_RES + x].Y;
					pt.z = realWorld[y * XN_VGA_X_RES + x].Z;
					//
					////cloud.points[y * XN_VGA_X_RES + x] = pt;
					cloud.push_back(pt);
					//cloud.push_back(pcl::PointXYZ(realWorld[y * XN_VGA_X_RES + x].X,-realWorld[y * XN_VGA_X_RES + x].Y,realWorld[y * XN_VGA_X_RES + x].Z));
			//		ac++;
				}
			} 
		}
		//viewer.removePointCloud("cloud"); 
		//viewer.addPointCloud(cloud.makeShared(),"cloud");
		//viewer.spinOnce (100);
		//mtx_2.unlock();
		viewer.showCloud(cloud.makeShared());

		printf("New Cloud, %d\n",ac%2);
	}
}

void kinect_thread_function(){
	while(running){
		XnStatus rc = XN_STATUS_OK;
		
		// Read a new frame
		rc = _context.WaitAnyUpdateAll();
		if (rc != XN_STATUS_OK)
		{
			printf("Read failed: %s\n", xnGetStatusString(rc));
			break;
		}

		mtx_.lock();
		
		_depth.GetMetaData(_depthMD);
		_image.GetMetaData(_imageMD);
		//xn_scene.GetMetaData(_sceneMD);
		
		cv::Mat depthMat16UC1(480, 640,CV_16UC1, (void*) _depthMD.Data());
		depthMat16UC1.convertTo(depthMat8UC1, CV_8UC1,0.05);

		cv::Mat color(480,640,CV_8UC3,(void*) _imageMD.Data());
		cv::cvtColor(color,color2,CV_RGB2BGR);

		mtx_.unlock();
	}
}



int main_thread_viewer(int argc, char* argv[]){
	//Context _context;
	//ScriptNode _scriptNode;
	////DepthGenerator _depth;
	//ImageGenerator _image;
	//DepthMetaData _depthMD;
	//ImageMetaData _imageMD;
	//SceneMetaData _sceneMD;

	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

	cv::namedWindow("Depth");
	xn::SceneAnalyzer xn_scene;
	bool remove_floor = false;
	XnPlane3D floorCoords;
    XnPoint3D floorPoint;
	//XnPoint3D * pointList = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 
	//XnPoint3D * realWorld = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 

	XnStatus rc;
	
	{
	EnumerationErrors errors;
	//rc = _context.InitFromXmlFile(SAMPLE_XML_PATH, _scriptNode, &errors);
	rc = _context.Init();
	
#ifdef _CCG
	//_context.OpenFileRecording("C:\\Dev\\Walkys\\Project\\Data\\Boxes.oni");
#endif

#ifdef _HOME
	_context.OpenFileRecording("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni"); 
#endif
	if (rc == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return (rc);
	}
	else if (rc != XN_STATUS_OK)
	{
		printf("Open failed: %s\n", xnGetStatusString(rc));
		return (rc);
	}

	rc = _context.FindExistingNode(XN_NODE_TYPE_DEPTH, _depth);
	if (rc != XN_STATUS_OK)
	{
		rc = _depth.Create(_context);

		if (rc != XN_STATUS_OK)
		{
			printf("No depth node exists! Check your XML.");
			return 1;
		}
	}

	rc = _context.FindExistingNode(XN_NODE_TYPE_IMAGE, _image);
	if (rc != XN_STATUS_OK)
	{
		rc = _image.Create(_context);

		if (rc != XN_STATUS_OK)
		{
			printf("No image node exists! Check your XML.");
			return 1;
		}
	}

	// Hybrid mode isn't supported in this sample
	if (_imageMD.FullXRes() != _depthMD.FullXRes() || _imageMD.FullYRes() != _depthMD.FullYRes())
	{
		printf ("The device depth and image resolution must be equal!\n");
		return 1;
	}

	// RGB is the only image format supported.
	if (_imageMD.PixelFormat() != XN_PIXEL_FORMAT_RGB24)
	{
		printf("The device image format must be RGB24\n");
		return 1;
	}

	//Sync Images
	_depth.GetAlternativeViewPointCap().SetViewPoint(_image);

	_context.StartGeneratingAll();
	}
	cv::RNG rng(12345);
	char ch = 0;


	_depth.GetMetaData(_depthMD);
	_depthMD_copy.CopyFrom(_depthMD);
	_image.GetMetaData(_imageMD);

	cv::Mat depthMat16UC1(480, 640,CV_16UC1, (void*) _depthMD.Data());
	depthMat16UC1.convertTo(depthMat8UC1, CV_8UC1,0.05);

	cv::Mat color(480,640,CV_8UC3,(void*) _imageMD.Data());
	cv::cvtColor(color,color2,CV_RGB2BGR);


	//pcl::visualization::CloudViewer viewer("Simple cloud_file Viewer");
	//pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width = 640*480;
	cloud.height = 1;
	cloud.points.resize (cloud.width * cloud.height);
	
	boost::thread kinectThread(kinect_thread_function);
	//Sleep(100);
	boost::thread pclThread(pcl_thread_function);
	
	while(running){
		cv::Mat clr;
		cv::Mat dep;

		mtx_.lock();
		
		color2.copyTo(clr);
		depthMat8UC1.copyTo(dep);
		
		mtx_.unlock();

		cv::imshow("Color", clr);
		cv::imshow("depth", dep);

		ch = cv::waitKey(12);
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

	kinectThread.join();
	pclThread.join();
}