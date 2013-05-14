#include "TopView.h"

#include <XnOS.h>
#include <math.h>

#include <XnCppWrapper.h>
using namespace xn;

#include <opencv2\opencv.hpp>

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

		//cv::Mat mask; cv::threshold(depthMat8UC1,mask,1,255,CV_THRESH_BINARY);
		cv::Mat mask_cv;			
		cv::inRange(depthMat16UC1,_min_bar,_max_bar,mask_cv);
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
			for(int y=0; y<XN_VGA_Y_RES; y++) { 
				for(int x=0; x<XN_VGA_X_RES; x++) { 
					if(realWorld[y * XN_VGA_X_RES + x].Z > 0.0 && ptr[y * XN_VGA_X_RES + x]){
						float value1 = distanceToPlane(realWorld[y * XN_VGA_X_RES + x],a,b,c,d) ;
						float value2 = distanceToPlane(realWorld[y * XN_VGA_X_RES + x],a2,b2,c2,d2) ;
						if( value1 < _thresh &&  value2 < _thresh){
							ptr[y * XN_VGA_X_RES + x]=255;
							//double as = depthMat16UC1.ptr<float>(x)[y];
							//printf("");
							//ptr2[y * XN_VGA_X_RES + x] = (uchar)abs(value);
							//((uchar*)(mask.data + mask.step*y))[x]=0;
							//pointList[y * XN_VGA_X_RES + x].Y = y; 
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

			//----------------------------------------------------------------------------------------
			// TOPVIEW
			//----------------------------------------------------------------------------------------
			//for(int y=0; y<XN_VGA_Y_RES; y++) { 
			//	for(int x=0; x<XN_VGA_X_RES; x++) { 
			//		XnPoint3D point1;
			//		point1.X = x; 
			//		point1.Y = y; 
			//		point1.Z = _depthMD[y * XN_VGA_X_RES + x]; 

			//		pointList[y * XN_VGA_X_RES + x] = point1;
			//	}
			//} 

			//_depth.ConvertProjectiveToRealWorld(XN_VGA_Y_RES*XN_VGA_X_RES, pointList, realWorld); 

			int width_min = 0; 
			int width_max = 640; 
			int height_min = _min_bar; 
			int height_max = _max_bar; 
			int width_total = width_max - width_min; 
			int height_total = height_max - height_min; 

			cv::Mat1b top_view(height_total, width_total, (const uchar)0);

			//double mm, nn;
			//cv::minMaxLoc(depth_masked,&mm,&nn);

			uchar* depth_ptr = mask_cv.data;
			uchar* top_view_ptr = top_view.data;

			//int xx = 0;
			int yy = 0;
			int value = 0;

			for(int y = 0; y < XN_VGA_Y_RES ; y++) { 
				for(int x = 0; x < XN_VGA_X_RES ; x++) {
					if(depth_ptr[y * XN_VGA_X_RES + x]){
						yy = realWorld[y * XN_VGA_X_RES + x].Z - height_min; //_depthMD[y * XN_VGA_X_RES + x];
						top_view_ptr[yy * XN_VGA_X_RES + x] = (uchar)255;
					}
				}
			}
			cv::imshow("TopView",top_view);
			//----------------------------------------------------------------------------------------
			// END TOPVIEW
			//----------------------------------------------------------------------------------------
			cv::Mat mat1, mat2, mat3, mat4;
			cv::dilate(top_view,mat1,cv::Mat(_kernel/2.0,_kernel/2.0,CV_8UC1));
			cv::erode(mat1,mat2,cv::Mat(_kernel/1.5,_kernel/1.5,CV_8UC1));

			cv::dilate(mat2,mat3,cv::Mat(_kernel,_kernel,CV_8UC1));
			mat3.copyTo(mat4);
			cv::Mat3b top_view_color(height_total, width_total, cv::Vec3i(0));

			cv::vector<cv::vector<cv::Point> > contours;
			cv::vector<cv::Vec4i> hierarchy;
			
			cv::findContours( mat4, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

			for(unsigned int i = 0; i< contours.size(); i++ ){
				if(contours[i].size() > 100){
					cv::Scalar clr = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
					cv::Rect rect = cv::boundingRect(contours[i]);
					drawContours( top_view_color, contours, i, clr, -1, 8, hierarchy, 0, cv::Point() );
					cv::rectangle(top_view_color,rect, clr);
				}
			}

			cv::imshow("Mat1",mat1);
			cv::imshow("Mat2",mat2);
			cv::imshow("Mat3",mat3);
			cv::imshow("CONTOURS",top_view_color);
		}

		cv::Mat3b cor(480,640);
		double min = _min_bar,max = _max_bar;
		compute_color_encoded_depth(depthMat16UC1,cor,&min,&max);

		//cor.copyTo(color3,mask_cv);
		depthMat8UC1.copyTo(color3,mask_cv);
		//cv::vector<cv::vector<cv::Point> > contours;
		//cv::vector<cv::Vec4i> hierarchy;

		//cv::findContours( mask_cv, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

		//for(unsigned int i = 0; i< contours.size(); i++ ){
		//	if(contours[i].size() > 100){
		//		cv::Scalar clr = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		//		cv::Rect rect = cv::boundingRect(contours[i]);

		//		cv::Moments mm = moments( contours[i], false );
		//		cv::Point2f center = cv::Point2f((float)(mm.m10/mm.m00) , (float)(mm.m01/mm.m00));

		//		drawContours( color3, contours, i, clr, 2, 8, hierarchy, 0, cv::Point() );

		//		cv::circle(color3,center,3,clr);
		//	}
		//}

		//cv::circle(color3, cv::Point(320,350),5,cv::Scalar(255,0,0));

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

					remove_floor = true;
				}
				else
					printf("Read failed: %s\n", xnGetStatusString(rc));
			}
		}
	}
}