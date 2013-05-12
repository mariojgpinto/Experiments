#include "RemoveFloor.h"

#include <XnOS.h>
#include <math.h>

#include <XnCppWrapper.h>
using namespace xn;

#include <opencv2\opencv.hpp>

#define SAMPLE_XML_PATH "C:\\Dev\\External\\OpenNI\\Data\\SamplesConfig.xml"

void compute_color_encoded_depth(const cv::Mat1f& depth_im, cv::Mat3b& color_depth_im,
                                     double* i_min_val, double* i_max_val);

float distanceToPlane(const XnPoint3D& p, float a, float b, float c, float d);

int main_remove_floor(int argc, char* argv[]){
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

	XnStatus rc;

	int _min_bar = 400;
	int _max_bar = 1500;
	int _thresh = 20;
	cv::namedWindow("Ranged Image");
	cv::createTrackbar("MinDepth", "Ranged Image", &_min_bar, 5000, NULL);
	cv::createTrackbar("MaxDepth", "Ranged Image", &_max_bar, 5000, NULL);
	cv::createTrackbar("FloorThresh", "Ranged Image", &_thresh, 100, NULL);
	{
	EnumerationErrors errors;
	rc = _context.InitFromXmlFile(SAMPLE_XML_PATH, _scriptNode, &errors);
	//rc = _context.Init();
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
		//cv::Mat depthMat8UC1;
		//depthMat16UC1.convertTo(depthMat8UC1, CV_8UC1,0.05);

		cv::Mat color(480,640,CV_8UC3,(void*) _imageMD.Data());
		cv::Mat color2;
		cv::cvtColor(color,color2,CV_RGB2BGR);

		//cv::Mat mask; cv::threshold(depthMat8UC1,mask,1,255,CV_THRESH_BINARY);
		cv::Mat mask_cv;			
		cv::inRange(depthMat16UC1,_min_bar,_max_bar,mask_cv);
		cv::Mat color3;
		cv::Mat color4;

		//int XN_VGA_Y_RES = 480;
		//int XN_VGA_X_RES = 640;
		if(remove_floor){
			
			//XnPoint3D point1;
			XnPoint3D point2;
			//for(int y=0; y<XN_VGA_Y_RES; y++) { 
			//	for(int x=0; x<XN_VGA_X_RES; x++) { 
			//		point1.X = x; 
			//		point1.Y = y; 
			//		point1.Z = (short) _depthMD[y * XN_VGA_X_RES + x]; 

			//		_depth.ConvertProjectiveToRealWorld(1, &point1, &point2); 

			//		float dist = distanceToPlane(point2,a,b,c,d);
			//		if(dist < 20){
			//			((uchar*)(mask.data + mask.step*y))[x]=0;
			//		}
			//	} 
			//} 
			uchar* ptr = mask_cv.data;
			for(int y=0; y<XN_VGA_Y_RES; y++) { 
				for(int x=0; x<XN_VGA_X_RES; x++) { 
					XnPoint3D point1;
					point1.X = x; 
					point1.Y = y; 
					point1.Z = _depthMD[y * XN_VGA_X_RES + x]; 

					pointList[y * XN_VGA_X_RES + x] = point1;

					//_depth.ConvertProjectiveToRealWorld(1, &point1, &point2); 

					//float dist = distanceToPlane(point2,a,b,c,d);
					//if(dist < 20){
					//	ptr[y * XN_VGA_X_RES + x]=0;
					//}
				}
			} 
			//XnVector3D realWorld[XN_VGA_Y_RES*XN_VGA_X_RES]; 

			_depth.ConvertProjectiveToRealWorld(XN_VGA_Y_RES*XN_VGA_X_RES, pointList, realWorld); 

			uchar* ptr2 = depthMat16UC1.data;
			for(int y=0; y<XN_VGA_Y_RES; y++) { 
				for(int x=0; x<XN_VGA_X_RES; x++) {
					if(pointList[y * XN_VGA_X_RES + x].Z > 0.0){
						float value = distanceToPlane(realWorld[y * XN_VGA_X_RES + x],a,b,c,d) ;
						if( value < _thresh){
							//ptr[y * XN_VGA_X_RES + x]=0;
							double as = depthMat16UC1.ptr<float>(x)[y];
							printf("");
							//ptr2[y * XN_VGA_X_RES + x] = (uchar)abs(value);
							//((uchar*)(mask.data + mask.step*y))[x]=0;
							//pointList[y * XN_VGA_X_RES + x].Y = y; 
						}
					}
					else{
						printf("");
					}
				} 
			} 
			////const XnDepthPixel* depth_data = _depthMD.Data();
			////XnPoint3D pt1; pt1.X = 320; pt1.Y = 240; pt1.Z = depth_data[pt1.Y*640+pt1.X];
			////XnPoint3D pt2;

			////xnConvertProjectiveToRealWorld(_depth,1,&pt1,&pt2);
			////
			////printf("");

			//
		}
		
		cv::Mat3b cor;
		double min = _min_bar,max = _max_bar;
		compute_color_encoded_depth(depthMat16UC1,cor,&min,&max);

		cor.copyTo(color3,mask_cv);

		cv::circle(color3, cv::Point(320,350),5,cv::Scalar(255,0,0));

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

		if(ch == ' '){
			cv::Mat temp; depthMat16UC1.copyTo(temp,mask_cv);
			cv::Mat depthMat8UC1;
			temp.convertTo(depthMat8UC1, CV_8UC1,0.05);

			static int n = 1;
			char buff[128];
			sprintf(buff,"Images\\img_%02d.png",n++);
			cv::imwrite(buff,depthMat8UC1);
			cv::imshow("depth",depthMat8UC1);
		}

		if(ch == 'f'){
			if(remove_floor){
				remove_floor = false;
			}
			else{
				rc = xn_scene.GetFloor( floorCoords);
				//rc = xnGetFloor(xn_scene, &floorCoords);
				if(rc == XN_STATUS_OK){
					floorPoint = floorCoords.ptPoint;

					std::cout << floorCoords.vNormal.X << " " << floorCoords.vNormal.Y << " " << floorCoords.vNormal.Z << std::endl;
					std::cout << floorCoords.ptPoint.X << " " << floorCoords.ptPoint.Y << " " << floorCoords.ptPoint.Z  << "\n" << std::endl;

					a = floorCoords.vNormal.X;
					b = floorCoords.vNormal.Y;
					c = floorCoords.vNormal.Z;

					XnPoint3D pt1; pt1.X = 320; pt1.Y = 350; pt1.Z = _depthMD[ 350*640+ 320 ];
					XnPoint3D pt2;
					XnPoint3D pt3;
					//_depth.ConvertProjectiveToRealWorld(1,&floorCoords.ptPoint,&pt2);
					_depth.ConvertProjectiveToRealWorld(1,&pt1,&pt2);
					_depth.ConvertRealWorldToProjective(1,&floorCoords.ptPoint,&pt3);


					d = -(a*floorCoords.ptPoint.X + b*floorCoords.ptPoint.Y + c*floorCoords.ptPoint.Z);

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