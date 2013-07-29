#include "TopView.h"

#include <XnOS.h>
#include <math.h>

#include <XnCppWrapper.h>
using namespace xn;

#include <opencv2\opencv.hpp>
#include <stdint.h>

#define SAMPLE_XML_PATH "C:\\Dev\\External\\OpenNI\\Data\\SamplesConfig.xml"

void compute_color_encoded_depth(const cv::Mat1f& depth_im, cv::Mat3b& color_depth_im,
                                     double* i_min_val, double* i_max_val);
float distanceToPlane(const XnPoint3D& p, float a, float b, float c, float d);




int main_top_view(int argc, char* argv[]){
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

	int _min_bar = 500;
	int _max_bar = 1000;
	int _thresh = 400;
	int _thresh_floor = 185;
	int _floor_range = 10;
	int _kernel = 253;
	cv::namedWindow("Ranged Image");
	cv::createTrackbar("MinDepth", "Ranged Image", &_min_bar, 10000, NULL);
	cv::createTrackbar("MaxDepth", "Ranged Image", &_max_bar, 10000, NULL);
	cv::createTrackbar("FloorHeight", "Ranged Image", &_thresh, 2500, NULL);
	cv::createTrackbar("FloorThresh", "Ranged Image", &_floor_range, 500, NULL);
	cv::createTrackbar("Kernel", "Ranged Image", &_kernel, 255, NULL);

	{
	EnumerationErrors errors;
	//rc = _context.InitFromXmlFile(SAMPLE_XML_PATH, _scriptNode, &errors);
	rc = _context.Init();
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

	//rc = _context.OpenFileRecording("C:\\Dev\\Walkys\\Project\\Data\\foot_1_left.oni");

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
	
	cv::Mat _matrix;
	cv::Mat _inverse;

	//Create and initialize auxiliar variables
	float** _temp_x;
	float** _temp_y;
	float** _temp_z;

	_temp_x = (float**)malloc(sizeof(float*) * 480);
	for(int i = 0 ; i < 480 ; i++){
		_temp_x[i] = (float*)malloc(sizeof(float) * 640);
	}

	_temp_y = (float**)malloc(sizeof(float*) * 480);
	for(int i = 0 ; i < 480 ; i++){
		_temp_y[i] = (float*)malloc(sizeof(float) * 640);
	}

	_temp_z = (float**)malloc(sizeof(float*) * 480);
	for(int i = 0 ; i < 480 ; i++){
		_temp_z[i] = (float*)malloc(sizeof(float) * 640);
	}
	

	float _max_xx = -FLT_MAX;
	float _max_yy = -FLT_MAX;
	float _min_xx = FLT_MAX;
	float _min_yy = FLT_MAX;
	int _width;
	int _height;

	double scale;
	
	cv::RNG rng(12345);
	char ch = 0;

//	XnStatus rc = XN_STATUS_OK;

	// Read a new frame
	rc = _context.WaitAnyUpdateAll();
	if (rc != XN_STATUS_OK)
	{
		printf("Read failed: %s\n", xnGetStatusString(rc));
	
	}

	_depth.GetMetaData(_depthMD);
	_image.GetMetaData(_imageMD);
	//xn_scene.GetMetaData(_sceneMD);

	while((ch = cv::waitKey(23)) != 27){
		//XnStatus rc = XN_STATUS_OK;

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

		cv::Mat mask_cv;			
		cv::inRange(depthMat16UC1,_min_bar,_max_bar,mask_cv);

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

					XnPoint3D pt1; pt1.X = 320; pt1.Y = 350; pt1.Z = _depthMD[ 350*640+ 320 ];
					XnPoint3D pt2;
					XnPoint3D pt3;
					//_depth.ConvertProjectiveToRealWorld(1,&floorCoords.ptPoint,&pt2);
					//_depth.ConvertProjectiveToRealWorld(1,&pt1,&pt2);
					_depth.ConvertRealWorldToProjective(1,&floorCoords.ptPoint,&pt3);

					d = -(a*floorCoords.ptPoint.X + b*floorCoords.ptPoint.Y + c*floorCoords.ptPoint.Z);

					floorPoint = floorCoords.ptPoint;
					floorPoint.Z-=_thresh_floor;

					a2 = floorCoords.vNormal.X;
					b2 = floorCoords.vNormal.Y;
					c2 = floorCoords.vNormal.Z;

					//_depth.ConvertRealWorldToProjective(1,&floorPoint,&pt2);

					d2 = -(a2*floorPoint.X + b2*floorPoint.Y + c2*floorPoint.Z);

					cv::Vec3f vec_1(1,0,0);
					cv::Vec3f vec_2 = vec_1.cross(cv::Vec3f(a,b,c));

					_matrix.create(3,3,CV_64FC1);
					_matrix.ptr<double>(0)[0] = vec_1.val[0];
					_matrix.ptr<double>(0)[1] = vec_1.val[1];
					_matrix.ptr<double>(0)[2] = vec_1.val[2];
					_matrix.ptr<double>(1)[0] = -a;
					_matrix.ptr<double>(1)[1] = -b;
					_matrix.ptr<double>(1)[2] = -c;
					_matrix.ptr<double>(2)[0] = vec_2.val[0];
					_matrix.ptr<double>(2)[1] = vec_2.val[1];
					_matrix.ptr<double>(2)[2] = vec_2.val[2];

					_inverse = _matrix.inv(1);

					remove_floor = true;
				}
				else
					printf("Read failed: %s\n", xnGetStatusString(rc));
			}
		}

		//cv::inRange(diff,_min_bar,_max_bar,mask_cv);
		cv::Mat color3;
		cv::Mat color4;

		//double mm, nn;
		//cv::Mat asd;
		//depthMat16UC1.copyTo(asd,mask_cv);
		//cv::minMaxLoc(asd,&mm,&nn);

		//int XN_VGA_Y_RES = 480;
		//int XN_VGA_X_RES = 640;
		if(remove_floor){
			XnPoint3D point2;

			uchar* ptr = mask_cv.data;
			for(int y=0; y<XN_VGA_Y_RES; y+=1) { 
				for(int x=0; x<XN_VGA_X_RES; x+=1) { 
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

			//for(int y=0; y<XN_VGA_Y_RES; y+=1) { 
			//	for(int x=0; x<XN_VGA_X_RES; x+=1) { 
			//		if(realWorld[y * XN_VGA_X_RES + x].Z > 0.0 && ptr[y * XN_VGA_X_RES + x]){
			//			float value1 = distanceToPlane(realWorld[y * XN_VGA_X_RES + x],a,b,c,d) ;
			//			float value2 = distanceToPlane(realWorld[y * XN_VGA_X_RES + x],a2,b2,c2,d2) ;
			//			if( value1 < _thresh &&  value2 < _thresh){
			//				ptr[y * XN_VGA_X_RES + x]=255;
			//				
			//				_temp_x[y][x] = _inverse.row(0).at<double>(0) * realWorld[y * XN_VGA_X_RES + x].X + 
			//								_inverse.row(0).at<double>(1) * realWorld[y * XN_VGA_X_RES + x].Y +
			//								_inverse.row(0).at<double>(2) * realWorld[y * XN_VGA_X_RES + x].Z;

			//				_temp_y[y][x] = _inverse.row(1).at<double>(0) * realWorld[y * XN_VGA_X_RES + x].X + 
			//								_inverse.row(1).at<double>(1) * realWorld[y * XN_VGA_X_RES + x].Y +
			//								_inverse.row(1).at<double>(2) * realWorld[y * XN_VGA_X_RES + x].Z;

			//				_temp_z[y][x] = _inverse.row(2).at<double>(0) * realWorld[y * XN_VGA_X_RES + x].X + 
			//								_inverse.row(2).at<double>(1) * realWorld[y * XN_VGA_X_RES + x].Y +
			//								_inverse.row(2).at<double>(2) * realWorld[y * XN_VGA_X_RES + x].Z;

			//				if(_temp_x[y][x] < _min_xx)
			//					_min_xx = _temp_x[y][x];
			//				if(_temp_x[y][x] > _max_xx)
			//					_max_xx = _temp_x[y][x];
			//				if(_temp_z[y][x] < _min_yy)
			//					_min_yy = _temp_z[y][x];
			//				if(_temp_z[y][x] > _max_yy)
			//					_max_yy = _temp_z[y][x];

			//				//realWorld[y * XN_VGA_X_RES + x].X += value2 * a2;
			//				//realWorld[y * XN_VGA_X_RES + x].Y += value2 * b2;
			//				//realWorld[y * XN_VGA_X_RES + x].Z += value2 * c2;
			//			}
			//			else{
			//				ptr[y * XN_VGA_X_RES + x]=0;

			//				_temp_x[y][x] = 0;
			//				_temp_y[y][x] = 0;
			//				_temp_z[y][x] = 0;
			//			}
			//		}
			//		else{
			//			_temp_x[y][x] = 0;
			//			_temp_y[y][x] = 0;
			//			_temp_z[y][x] = 0;
			//		}
			//	} 
			//} 

			//----------------------------------------------------------------------------------------
			// TOPVIEW
			//----------------------------------------------------------------------------------------
			_max_xx = -FLT_MAX;
			_max_yy = -FLT_MAX;
			_min_xx = FLT_MAX;
			_min_yy = FLT_MAX;
			
			for(int y=0; y<XN_VGA_Y_RES; y+=1) { 
				for(int x=0; x<XN_VGA_X_RES; x+=1) { 
			//for(int y=0; y<XN_VGA_Y_RES; y+=1) { 
			//	for(int x=0; x<XN_VGA_X_RES; x+=1) { 
					if(realWorld[y * XN_VGA_X_RES + x].Z > 0.0 && ptr[y * XN_VGA_X_RES + x]){
						float value1 = distanceToPlane(realWorld[y * XN_VGA_X_RES + x],a,b,c,d) ;
						float value2 = distanceToPlane(realWorld[y * XN_VGA_X_RES + x],a2,b2,c2,d2) ;
						if( value1 < _thresh &&  value2 < _thresh){
							ptr[y * XN_VGA_X_RES + x]=255;
							
							_temp_x[y][x] = _inverse.row(0).at<double>(0) * realWorld[y * XN_VGA_X_RES + x].X + 
											_inverse.row(0).at<double>(1) * realWorld[y * XN_VGA_X_RES + x].Y +
											_inverse.row(0).at<double>(2) * realWorld[y * XN_VGA_X_RES + x].Z;

							_temp_y[y][x] = _inverse.row(1).at<double>(0) * realWorld[y * XN_VGA_X_RES + x].X + 
											_inverse.row(1).at<double>(1) * realWorld[y * XN_VGA_X_RES + x].Y +
											_inverse.row(1).at<double>(2) * realWorld[y * XN_VGA_X_RES + x].Z;

							_temp_z[y][x] = _inverse.row(2).at<double>(0) * realWorld[y * XN_VGA_X_RES + x].X + 
											_inverse.row(2).at<double>(1) * realWorld[y * XN_VGA_X_RES + x].Y +
											_inverse.row(2).at<double>(2) * realWorld[y * XN_VGA_X_RES + x].Z;

							if(_temp_x[y][x] < _min_xx)
								_min_xx = _temp_x[y][x];
							if(_temp_x[y][x] > _max_xx)
								_max_xx = _temp_x[y][x];
							if(_temp_z[y][x] < _min_yy)
								_min_yy = _temp_z[y][x];
							if(_temp_z[y][x] > _max_yy)
								_max_yy = _temp_z[y][x];
						}
						else{
							ptr[y * XN_VGA_X_RES + x]=0;

							_temp_x[y][x] = 0;
							_temp_y[y][x] = 0;
							_temp_z[y][x] = 0;
						}
					}
					else{
						_temp_x[y][x] = 0;
						_temp_y[y][x] = 0;
						_temp_z[y][x] = 0;
					}
				} 
			}

			_width = _max_xx - _min_xx;
			_height = _max_yy - _min_yy;

			scale = (_max_bar > 3000) ? 10.0 : (_max_bar > 1500) ? 5.0 : 2.0;

			_width = _max_bar/scale;
			_height = _max_bar/scale;
			_min_yy = _min_bar;
			_min_xx = -_max_bar/2.0;

			cv::Mat1b top = cv::Mat::zeros(cv::Size(_width,_height),CV_8UC1);
			uchar* top_ptr = top.data;

			int _x,_y;
			for(int y=0; y<XN_VGA_Y_RES; y+=1) { 
				for(int x=0; x<XN_VGA_X_RES; x+=1) { 
					_x = (int)((_temp_x[y][x] - _min_xx)/scale);
					_y = (int)((_temp_z[y][x] - _min_yy)/scale);
					if(_y >= 0 && _x >= 0 && _x < _width && _y < _height){
						top_ptr[_x * _width + _y] = 255;
					}
				}
			}

			//cv::erode(top,top,cv::Mat(3,3,CV_8UC1));
			//cv::dilate(top,top,cv::Mat(5,5,CV_8UC1));
			//cv::blur(top,top,cv::Size(5,5));

			cv::imshow("top",top);
		}

		cv::Mat3b cor(480,640);
		double min = _min_bar,max = _max_bar;
		compute_color_encoded_depth(depthMat16UC1,cor,&min,&max);

		cor.copyTo(color3,mask_cv);

		cv::imshow("Ranged Image",color3);
		//cv::imshow("Depth Image",depthMat8UC1);
		cv::imshow("Color Image",color2);
		
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
	}
}