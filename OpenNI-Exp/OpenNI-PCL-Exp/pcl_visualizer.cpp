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

#define SAMPLE_XML_PATH "C:\\Dev\\External\\OpenNI\\Data\\SamplesConfig.xml"

void compute_color_encoded_depth(const cv::Mat1f& depth_im, cv::Mat3b& color_depth_im,
                                     double* i_min_val, double* i_max_val);
float distanceToPlane(const XnPoint3D& p, float a, float b, float c, float d);


int main_pcl_visualizer(int argc, char* argv[]){
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
	float a = 0,b = 0,c = 0,d = 0;
	float a2 = 0,b2 = 0,c2 = 0,d2 = 0;

	XnStatus rc;

	int _min_bar = 400;
	int _max_bar = 1200;
	int _thresh = 175;
	int _thresh_floor = 185;
	int _floor_range = 10;
	int _kernel = 9;
	cv::namedWindow("Ranged Image");
	cv::createTrackbar("MinDepth", "Ranged Image", &_min_bar, 5000, NULL);
	cv::createTrackbar("MaxDepth", "Ranged Image", &_max_bar, 5000, NULL);
	cv::createTrackbar("FloorHeight", "Ranged Image", &_thresh, 200, NULL);
	cv::createTrackbar("FloorThresh", "Ranged Image", &_floor_range, 200, NULL);
	cv::createTrackbar("Kernel", "Ranged Image", &_kernel, 21, NULL);

	{
	EnumerationErrors errors;
	//rc = _context.InitFromXmlFile(SAMPLE_XML_PATH, _scriptNode, &errors);
	rc = _context.Init();
	_context.OpenFileRecording("C:\\Dev\\Walkys\\Project\\Data\\Boxes.oni");

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

	rc = xn_scene.Create(_context);
//rc = _context.FindExistingNode(XN_NODE_TYPE_SCENE, xn_scene);
	if (rc != XN_STATUS_OK)
	{
		printf("No Scene Analyzer\n.");
		getchar();
		return 1;
	}

	xn::UserGenerator _user;
	rc = _user.Create(_context);
	//rc = _context.FindExistingNode(XN_NODE_TYPE_SCENE, xn_scene);
    if (rc != XN_STATUS_OK)
    {
		printf("No _User Analyzer\n.");
		getchar();
		return 1;
    }
	
	//xn_scene.GetMetaData(_sceneMD);

	_context.StartGeneratingAll();
	}
	cv::RNG rng(12345);
	char ch = 0;


	pcl::visualization::CloudViewer viewer("Simple cloud_file Viewer");
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	cloud.width = 640*480;
	cloud.height = 1;
	cloud.points.resize (cloud.width * cloud.height);

	int asc = cloud.size();

	cloud.clear();

	asc = cloud.size();

	bool rea = cloud.empty();

	

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
		cv::Mat color2;
		cv::cvtColor(color,color2,CV_RGB2BGR);
		cv::imshow("Color", color2);
		//cv::Mat mask; cv::threshold(depthMat8UC1,mask,1,255,CV_THRESH_BINARY);
		cv::Mat mask_cv;			
		cv::inRange(depthMat16UC1,_min_bar,_max_bar,mask_cv);
		cv::Mat color3;
		cv::Mat color4;

		if(remove_floor){
			XnPoint3D point2;

			uchar* ptr = mask_cv.data;
			
			for(int y=0; y<XN_VGA_Y_RES; y++) { 
				for(int x=0; x<XN_VGA_X_RES; x++) { 
					XnPoint3D point1;
					point1.X = x; 
					point1.Y = y; 
					point1.Z = _depthMD[y * XN_VGA_X_RES + x]; 

					pointList[y * XN_VGA_X_RES + x] = point1;
				}
			} 

			_depth.ConvertProjectiveToRealWorld(XN_VGA_Y_RES*XN_VGA_X_RES, pointList, realWorld); 

			_thresh_floor = _thresh + _floor_range;

			floorPoint = floorCoords.ptPoint;
			floorPoint.X += floorCoords.vNormal.X * _thresh_floor;
			floorPoint.Y += floorCoords.vNormal.Y * _thresh_floor;
			floorPoint.Z += floorCoords.vNormal.Z * _thresh_floor;
			//floorPoint.Z -= _thresh_floor;
			a2 = floorCoords.vNormal.X;
			b2 = floorCoords.vNormal.Y;
			c2 = floorCoords.vNormal.Z;
			d2 = -(a2*floorPoint.X + b2*floorPoint.Y + c2*floorPoint.Z);

			uchar* ptr2 = depthMat16UC1.data;
			int counter = 0;
			cloud.points.clear();

			uint8_t* ptr_clr = (uint8_t*)color.data;
			for(int y=0; y<XN_VGA_Y_RES; y++) { 
				//uchar* dssd = color.ptr<uchar>(y);
				for(int x=0; x<XN_VGA_X_RES; x++) { 
					if(realWorld[y * XN_VGA_X_RES + x].Z > 0.0 && ptr[y * XN_VGA_X_RES + x]){
						float value1 = distanceToPlane(realWorld[y * XN_VGA_X_RES + x],a,b,c,d) ;
						float value2 = distanceToPlane(realWorld[y * XN_VGA_X_RES + x],a2,b2,c2,d2) ;
						if( value1 < _thresh &&  value2 < _thresh){
							ptr[y * XN_VGA_X_RES + x]=255;

							//PCL
							//color.ptr<cv::Vec3b>()
							/*pcl::PointXYZRGB pt(color.at<cv::Vec3b>(y,x)[0],
												color.at<cv::Vec3b>(y,x)[1],
												color.at<cv::Vec3b>(y,x)[2]);*/
							//pcl::PointXYZRGB pt(dssd[3 * x + 0],
							//					dssd[3 * x + 1],
							//					dssd[3 * x + 2]);												
							pcl::PointXYZRGB pt(ptr_clr[y * XN_VGA_X_RES * 3 + x* 3 + 0],
												ptr_clr[y * XN_VGA_X_RES * 3 + x* 3 + 1],
												ptr_clr[y * XN_VGA_X_RES * 3 + x* 3 + 2]);
							pt.x = realWorld[y * XN_VGA_X_RES + x].X;
							pt.y = -realWorld[y * XN_VGA_X_RES + x].Y;
							pt.z = realWorld[y * XN_VGA_X_RES + x].Z;
							cloud.push_back(pcl::PointXYZRGB(pt));
						}
						else{
							ptr[y * XN_VGA_X_RES + x]=0;
						}
					}
					else{
						printf("");
					}
				} 
			}

			viewer.showCloud(cloud.makeShared());
		}

		cv::Mat3b cor(480,640);
		double min = _min_bar,max = _max_bar;
		compute_color_encoded_depth(depthMat16UC1,cor,&min,&max);

		//cor.copyTo(color3,mask_cv);
		depthMat8UC1.copyTo(color3,mask_cv);

		//cv::imshow("depth",depthMat8UC1);
		cv::imshow("Ranged Image",color3);
		//cv::imshow("color",);

		++_frame_counter;
		if (_frame_counter == 15)
		{
			double current_tick = cv::getTickCount();
			_frame_rate = _frame_counter / ((current_tick - _last_tick)/cv::getTickFrequency());
			_last_tick = current_tick;
			_frame_counter = 0;
			printf("%.2f\n",_frame_rate);
		}

		if(ch == 'f'){
			if(remove_floor){
				remove_floor = false;
			}
			else{
				rc = xn_scene.GetFloor( floorCoords);
				//rc = xnGetFloor(xn_scene, &floorCoords);
				if(rc == XN_STATUS_OK){
					std::cout << floorCoords.vNormal.X << " " << floorCoords.vNormal.Y << " " << floorCoords.vNormal.Z << std::endl;
					std::cout << floorCoords.ptPoint.X << " " << floorCoords.ptPoint.Y << " " << floorCoords.ptPoint.Z  << "\n" << std::endl;

					a = floorCoords.vNormal.X;
					b = floorCoords.vNormal.Y;
					c = floorCoords.vNormal.Z;
					d = -(a*floorCoords.ptPoint.X + b*floorCoords.ptPoint.Y + c*floorCoords.ptPoint.Z);

					floorPoint = floorCoords.ptPoint;
					floorPoint.Z-=_thresh_floor;

					a2 = floorCoords.vNormal.X;
					b2 = floorCoords.vNormal.Y;
					c2 = floorCoords.vNormal.Z;
					d2 = -(a2*floorPoint.X + b2*floorPoint.Y + c2*floorPoint.Z);

					remove_floor = true;
				}
				else
					printf("Read failed: %s\n", xnGetStatusString(rc));
			}
		}
	}
}

float distanceToPlane(const XnPoint3D& p, float a, float b, float c, float d){
	float v = a*p.X + b*p.Y + c*p.Z + d;
	v /= sqrt(a*a+b*b+c*c);
	return std::abs(v);
}

void compute_color_encoded_depth(const cv::Mat1f& depth_im, cv::Mat3b& color_depth_im,
                                     double* i_min_val, double* i_max_val)
{
	double min_val, max_val;
	if (i_min_val && i_max_val)
	{
		min_val = *i_min_val;
		max_val = *i_max_val;
	}
	else
	{
		minMaxLoc(depth_im, &min_val, &max_val);
	}

	color_depth_im.create(depth_im.size());
	for (int r = 0; r < depth_im.rows; ++r)
	{
		const float* depth_data = depth_im.ptr<float>(r);
		cv::Vec3b* depth_color_data = color_depth_im.ptr<cv::Vec3b>(r);
		for (int c = 0; c < depth_im.cols; ++c)
		{
			int v = 255*6*(depth_data[c]-min_val)/(max_val-min_val);
			if (v < 0) v = 0;
			char r,g,b;
			int lb = v & 0xff;
			switch (v / 256) {
			case 0:
				r = 255;
				g = 255-lb;
				b = 255-lb;
				break;
			case 1:
				r = 255;
				g = lb;
				b = 0;
				break;
			case 2:
				r = 255-lb;
				g = 255;
				b = 0;
				break;
			case 3:
				r = 0;
				g = 255;
				b = lb;
				break;
			case 4:
				r = 0;
				g = 255-lb;
				b = 255;
				break;
			case 5:
				r = 0;
				g = 0;
				b = 255-lb;
				break;
			default:
				r = 0;
				g = 0;
				b = 0;
				break;
			}
			if (v == 0)
			{
				r = g = b = 0;
			}
			depth_color_data[c] = cv::Vec3b(b,g,r);
		}
	}
}









int main_pcl_visualizer_simple(int argc, char* argv[]){
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
}