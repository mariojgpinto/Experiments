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


void func_pcl(NIThreadedKinect* kinect, pcl::visualization::CloudViewer* viewer,bool *running){
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

int main(int argc, char* argv[]){
	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

	NIThreadedKinect* kinect = new NIThreadedKinect();
	
	//pcl::PointCloud<pcl::PointXYZ> cloud;
	//pcl::PointCloud<pcl::PointXYZ>* cloud_ptr;// = new pcl::PointCloud<pcl::PointXYZ>();

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

	//Init Kinect
	kinect->start_thread(NIThreadedKinect::CAPTURE_T);
	Sleep(100);
	kinect->start_thread(NIThreadedKinect::POINT_CLOUD_T);
	Sleep(100);

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

/*
int main(int argc, char* argv[]){
	Context _context;
	ScriptNode _scriptNode;
	DepthGenerator _depth;
	ImageGenerator _image;
	DepthMetaData _depthMD;
	ImageMetaData _imageMD;
	SceneMetaData _sceneMD;

	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;


	xn::SceneAnalyzer xn_scene;
	bool remove_floor = false;
	XnPlane3D floorCoords;
    XnPoint3D floorPoint;
	XnPoint3D * pointList = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 
	XnPoint3D * realWorld = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 

	XnStatus rc;
	
	{
	EnumerationErrors errors;
	//rc = _context.InitFromXmlFile(SAMPLE_XML_PATH, _scriptNode, &errors);
	rc = _context.Init();
	
#ifdef _CCG
	_context.OpenFileRecording("C:\\Dev\\Walkys\\Project\\Data\\Boxes.oni");
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

	_depth.GetMetaData(_depthMD);
	_image.GetMetaData(_imageMD);

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


	pcl::visualization::CloudViewer viewer("Simple cloud_file Viewer");
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width = 640*480;
	cloud.height = 1;
	cloud.points.resize (cloud.width * cloud.height);

	int yres = XN_VGA_Y_RES - 50;
	int xres = XN_VGA_X_RES - 100;
	while((ch = cv::waitKey(23)) != 27){
		XnStatus rc = XN_STATUS_OK;

		// Read a new frame
		rc = _context.WaitAnyUpdateAll();
		if (rc != XN_STATUS_OK)
		{
			printf("Read failed: %s\n", xnGetStatusString(rc));
			break;
		}

		_depth.GetMetaData(_depthMD);
		_image.GetMetaData(_imageMD);
		//xn_scene.GetMetaData(_sceneMD);

		cv::Mat depthMat16UC1(480, 640,CV_16UC1, (void*) _depthMD.Data());
		cv::Mat depthMat8UC1;
		depthMat16UC1.convertTo(depthMat8UC1, CV_8UC1,0.05);

		cv::Mat color(480,640,CV_8UC3,(void*) _imageMD.Data());
		//cv::Mat color2;
		//cv::cvtColor(color,color2,CV_RGB2BGR);
		//cv::imshow("Color", color2);
		////cv::Mat mask; cv::threshold(depthMat8UC1,mask,1,255,CV_THRESH_BINARY);
		//cv::Mat mask_cv;			
		//cv::inRange(depthMat16UC1,_min_bar,_max_bar,mask_cv);

		uchar* depth_ptr = depthMat16UC1.data;

		for(int y=0; y<yres; y++) { 
			for(int x=0; x<xres; x++) { 
				XnPoint3D point1;
				point1.X = x; 
				point1.Y = y; 
				point1.Z = _depthMD[y * XN_VGA_X_RES + x]; 

				pointList[y * XN_VGA_X_RES + x] = point1;
			}
		} 

		_depth.ConvertProjectiveToRealWorld(XN_VGA_Y_RES*XN_VGA_X_RES, pointList, realWorld); 

		cloud.points.clear();
		//int ac = 0;
		uint8_t* ptr_clr = (uint8_t*)color.data;
		for(int y=50; y<XN_VGA_Y_RES; y++) { 
			for(int x=100; x<XN_VGA_X_RES; x++) { 
				if(realWorld[y * XN_VGA_X_RES + x].Z > 0.0){
					//pcl::PointXYZRGB pt(ptr_clr[y * XN_VGA_X_RES * 3 + x* 3 + 0],
					//					ptr_clr[y * XN_VGA_X_RES * 3 + x* 3 + 1],
					//					ptr_clr[y * XN_VGA_X_RES * 3 + x* 3 + 2]);
					//pt.x = realWorld[y * XN_VGA_X_RES + x].X;
					//pt.y = -realWorld[y * XN_VGA_X_RES + x].Y;
					//pt.z = realWorld[y * XN_VGA_X_RES + x].Z;
					//
					////cloud.points[y * XN_VGA_X_RES + x] = pt;
					//cloud.push_back(pt);
					cloud.push_back(pcl::PointXYZ(realWorld[y * XN_VGA_X_RES + x].X,-realWorld[y * XN_VGA_X_RES + x].Y,realWorld[y * XN_VGA_X_RES + x].Z));
			//		ac++;
				}
			} 
		}

		viewer.showCloud(cloud.makeShared());

		
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
}*/