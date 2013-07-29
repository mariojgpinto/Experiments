#include "Walkys-PCL.h"

#include <XnOS.h>
#include <math.h>

#include <XnCppWrapper.h>
using namespace xn;

#include <opencv2\opencv.hpp>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>


#define SAMPLE_XML_PATH "C:\\Dev\\External\\OpenNI\\Data\\SamplesConfig.xml"

void compute_color_encoded_depth(const cv::Mat1f& depth_im, cv::Mat3b& color_depth_im,
                                     double* i_min_val, double* i_max_val);
float distanceToPlane(const XnPoint3D& p, float a, float b, float c, float d);



int main_walkys_top_view(int argc, char* argv[]);


#include <boost/asio.hpp>
using boost::asio::ip::tcp;

char* host = "169.254.43.139";
char* port = "13000";

//#define _SOCKETS

int main_walkys(int argc, char* argv[]){
	//return main_walkys_skeletonize(argc,argv);
	return main_walkys_top_view(argc,argv);
}



int main_walkys_top_view(int argc, char* argv[]){
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

	int _min_bar = 350;
	int _max_bar = 1000;
	int _thresh = 350;
	int _thresh_floor = 10;
	int _floor_range = 10;
	//int _kernel = 253;
	
	cv::namedWindow("Ranged Image");
	

	cv::createTrackbar("MinDepth", "Ranged Image", &_min_bar, 10000, NULL);
	cv::createTrackbar("MaxDepth", "Ranged Image", &_max_bar, 10000, NULL);
	cv::createTrackbar("FloorThresh", "Ranged Image", &_floor_range, 500, NULL);
	cv::createTrackbar("FloorHeight", "Ranged Image", &_thresh, 2500, NULL);
	

	bool _leg = true;
	int _leg_height_start = 110;
	int _leg_height_end = 20;

	cv::namedWindow("Leg");
	cv::createTrackbar("LegStart", "Leg", &_leg_height_start, 750, NULL);
	cv::createTrackbar("LegEnd", "Leg", &_leg_height_end, 100, NULL);

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

	rc = _context.OpenFileRecording("C:\\Dev\\Walkys\\Project\\Data\\Mirrors\\mirro_mirror_boxes.oni");
	_context.SetGlobalMirror(true);
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
	

	int ***_back_positions;
	_back_positions = (int***)malloc(sizeof(int**) * 500);
	for(int i = 0 ; i < 500 ; i++){
		_back_positions[i] = (int**)malloc(sizeof(int*) * 500);
		for(int j = 0 ; j < 500 ; j++){
			_back_positions[i][j] = (int*)malloc(sizeof(int) * 2);
			_back_positions[i][j][0] = 0;
			_back_positions[i][j][1] = 0;
		}
	}

	//3D
	bool _new_foot_info_3d = false;
	float _leg_3d_xx;		float _leg_3d_yy;		float _leg_3d_zz;
	float _heel_3d_xx;		float _heel_3d_yy;		float _heel_3d_zz;
	float _tiptoe_3d_xx;	float _tiptoe_3d_yy;	float _tiptoe_3d_zz;

	//2D
	bool _new_foot_info_2d = false;
	float _leg_2d_xx;		float _leg_2d_yy;
	float _heel_2d_xx;		float _heel_2d_yy;
	float _tiptoe_2d_xx;	float _tiptoe_2d_yy;


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

#ifdef _SOCKETS
//	try{
		boost::asio::io_service io_service;

		tcp::resolver resolver(io_service);
		tcp::resolver::query query(tcp::v4(), host, port);
		tcp::resolver::iterator iterator = resolver.resolve(query);

		tcp::socket s(io_service);
		boost::asio::connect(s, iterator);

		std::string message;
		int counter = 0;
	//}
	//catch(std::exception& e)
	//{
	//	std::cerr << "Exception: " << e.what() << "\n";
	//}
#endif

	pcl::visualization::PCLVisualizer viewer("Simple cloud_file Viewer");
	//pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width = 640*480;
	cloud.height = 1;
	cloud.points.resize (cloud.width * cloud.height);

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

		//PCL
		cloud.points.clear();
		viewer.removeAllPointClouds();
		viewer.removeAllShapes();

#ifdef _SOCKETS
		//Sockets
		message.clear();
		counter = 0;
#endif
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

					//XnPoint3D pt1; pt1.X = 320; pt1.Y = 350; pt1.Z = _depthMD[ 350*640+ 320 ];
					//XnPoint3D pt2;
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

		if(ch == 'l'){
			_leg = !_leg;
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
			//uchar* depth_ptr = (uchar*)depthMat8UC1.data;
			//cv::imshow("deptg",depthMat8UC1);
			for(int y=0; y<XN_VGA_Y_RES; y+=2) { 
				for(int x=0; x<XN_VGA_X_RES; x+=2) { 
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

			//----------------------------------------------------------------------------------------
			// TOPVIEW
			//----------------------------------------------------------------------------------------
			_max_xx = -FLT_MAX;
			_max_yy = -FLT_MAX;
			_min_xx = FLT_MAX;
			_min_yy = FLT_MAX;
			
			for(int y=0; y<XN_VGA_Y_RES; y+=2) { 
				for(int x=0; x<XN_VGA_X_RES; x+=2) { 
			//for(int y=0; y<XN_VGA_Y_RES; y+=2) { 
			//	for(int x=0; x<XN_VGA_X_RES; x+=2) { 
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

			for(int kx = 0 ; kx < 500 ; kx++){
				for(int ky = 0 ; ky < 500 ; ky++){
					_back_positions[kx][ky][0] = 0;
					_back_positions[kx][ky][1] = 0;
				}
			}
			
			int _x,_y;
			for(int y=0; y<XN_VGA_Y_RES; y+=2) { 
				for(int x=0; x<XN_VGA_X_RES; x+=2) { 
					_y = (int)((_temp_x[y][x] - _min_xx)/scale);
					_x = (int)((_temp_z[y][x] - _min_yy)/scale);
					if(_y >= 0 && _x >= 0 && _x < _width && _y < _height){
						top_ptr[_x * _width + _y] = 255;
						_back_positions[_x][_y][0] = x;
						_back_positions[_x][_y][1] = y;
						cloud.push_back(pcl::PointXYZ(realWorld[y * XN_VGA_X_RES + x].X,realWorld[y * XN_VGA_X_RES + x].Y,realWorld[y * XN_VGA_X_RES + x].Z));
					}
				}
			}
			cv::Mat top_w;
			cv::Mat top_w_1;
			cv::Mat top_w_2;
			cv::Mat top_w_3;
			cv::dilate(top,top_w,cv::Mat(3,3,CV_8UC1));
			cv::erode(top_w,top_w,cv::Mat(5,5,CV_8UC1));
			
			//MPcv::dilate(top_w,top_w_1,cv::Mat(7,7,CV_8UC1));
			cv::dilate(top_w,top_w_2,cv::Mat(7,15,CV_8UC1));
			//MPcv::dilate(top_w,top_w_3,cv::Mat(15,7,CV_8UC1));
			
			cv::dilate(top_w,top_w,cv::Mat(7,7,CV_8UC1));
			//cv::blur(top,top,cv::Size(5,5));

			//MPcv::imshow("7_7",top_w_1);
			//MPcv::imshow("7_11",top_w_2);
			//MPcv::imshow("11_7",top_w_3);


			cv::Mat top_countours = cv::Mat::zeros(top.size(),CV_8UC1);

			cv::vector<cv::vector<cv::Point> > contours;
			cv::vector<cv::Vec4i> hierarchy;
			
			cv::findContours( top_w_2, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

			XnPoint3D foot_point;
			uchar* ptr_cont = top.data;

			//if(_leg){
				//int _x,_y;
				cv::Mat leg_temp = cv::Mat::ones(color2.size(),CV_8UC1);
				uchar* ptr_leg = leg_temp.data;

				int ac = 0;
				double _mid_x = 0.0;
				double _mid_y = 0.0;
				double _mid_z = 0.0;

				int _mid_xx = 0;
				int _mid_yy = 0;
		

				for(int y=0; y<XN_VGA_Y_RES; y+=2) { 
					for(int x=0; x<XN_VGA_X_RES; x+=2) {
						if(_temp_y[y][x] > _leg_height_start && _temp_y[y][x] < _leg_height_start + _leg_height_end){
							ptr_leg[y * XN_VGA_X_RES + x] = 0;
							ac++;
							_mid_x += realWorld[y * XN_VGA_X_RES + x].X;
							_mid_y += realWorld[y * XN_VGA_X_RES + x].Y;
							_mid_z += realWorld[y * XN_VGA_X_RES + x].Z;
							_mid_xx += x;
							_mid_yy += y;
						}
					}
				}

				_mid_x /= ac;
				_mid_y /= ac;
				_mid_z /= ac;
				_mid_xx = (int)((float)_mid_xx/(float)ac);
				_mid_yy = (int)((float)_mid_yy/(float)ac);

				double p1 = _temp_x[_mid_yy][_mid_xx];
				double p2 = _temp_y[_mid_yy][_mid_xx];
				double p3 = _temp_z[_mid_yy][_mid_xx];

				XnPoint3D ppp[1]; 
				XnPoint3D ppp_out[1]; 
				ppp[0].X = _mid_x;
				ppp[0].Y = _mid_y;
				ppp[0].Z = _mid_z;


				//_depth.ConvertRealWorldToProjective(1,ppp,ppp_out);

				//cv::Mat color_temp; color2.copyTo(color_temp,leg_temp);

				//char buff1[100];
				//sprintf(buff1," real: %.2f , %.2f , %.2f", ppp[0].X , ppp[0].Y , ppp[0].Z);
				////char buff2[100];
				////sprintf(buff2," proj: %.2f , %.2f , %.2f", ppp_out[0].X , ppp_out[0].Y , ppp_out[0].Z);
				//char buff3[100];
				//sprintf(buff3," med : %.2f , %.2f , %.2f", p1 , p2 , p3);

				//cv::putText(color_temp, buff1, cvPoint(30 , 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 1., cvScalar(255,255,255), 1, CV_AA);
				//cv::putText(color_temp, buff3, cvPoint(30 , 60), cv::FONT_HERSHEY_COMPLEX_SMALL, 1., cvScalar(255,255,255), 1, CV_AA);
				//cv::putText(color_temp, buff2, cvPoint(30 , 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 1., cvScalar(255,255,255), 1, CV_AA);

				//cv::circle(color_temp,cv::Point(ppp_out[0].X, XN_VGA_Y_RES - ppp_out[0].Y),5,cv::Scalar(255,255,255),-1);
				//cv::circle(color_temp,cv::Point(_mid_xx, _mid_yy),5,cv::Scalar(255,255,255),-1);
				//cv::imshow("Leg",color_temp);

				int mxx = (int)((ppp[0].X - _min_xx)/scale);
				int myy = (int)((ppp[0].Z + 30 - _min_yy)/scale);	

				if(mxx >= 0 && myy >= 0){
					//MPcv::circle(top_countours,cv::Point(mxx, myy),5,cv::Scalar(50,50,50),-1);
					//MPcv::circle(top_countours,cv::Point(mxx, myy),2,cv::Scalar(255,255,255),-1);
				}

				XnPoint3D ptleg; ptleg.X = _mid_x; ptleg.Y = _mid_y; ptleg.Z = _mid_z;

				float leg_dist = distanceToPlane(ppp[0],a,b,c,d);
				char buff4[100];
				sprintf(buff4," Dist to Plane: %.2f", leg_dist);
				cv::putText(top, buff4, cvPoint(30 , 400), cv::FONT_HERSHEY_COMPLEX_SMALL, 1., cvScalar(255,255,255), 1, CV_AA);
				//cv::imshow("top",top);
				
				viewer.addSphere(pcl::PointXYZ(_mid_x,_mid_y,_mid_z+50),10,0,0,255,"Corte1");
				//viewer.addSphere(pcl::PointXYZ(_mid_x+(a*-leg_dist),_mid_y+(b*-leg_dist)+50,_mid_z+(c*-leg_dist)+50),10,"Leg2");
				{
					int _x,_y;

					int ac = 0;
					double _mid_x = 0.0;
					double _mid_y = 0.0;
					double _mid_z = 0.0;

					int _mid_xx = 0;
					int _mid_yy = 0;
		

					for(int y=0; y<XN_VGA_Y_RES; y+=2) { 
						for(int x=0; x<XN_VGA_X_RES; x+=2) {
							if(_temp_y[y][x] > _leg_height_start - 50 && _temp_y[y][x] < _leg_height_start + _leg_height_end - 50){
								ptr_leg[y * XN_VGA_X_RES + x] = 0;
								ac++;
								_mid_x += realWorld[y * XN_VGA_X_RES + x].X;
								_mid_y += realWorld[y * XN_VGA_X_RES + x].Y;
								_mid_z += realWorld[y * XN_VGA_X_RES + x].Z;
								_mid_xx += x;
								_mid_yy += y;
							}
						}
					}

					_mid_x /= ac;
					_mid_y /= ac;
					_mid_z /= ac;
					_mid_xx = (int)((float)_mid_xx/(float)ac);
					_mid_yy = (int)((float)_mid_yy/(float)ac);


					//cv::circle(color_temp,cv::Point(_mid_xx, _mid_yy),5,cv::Scalar(255,255,255),-1);
					//cv::imshow("Leg",color_temp);

					double p1 = _temp_x[_mid_yy][_mid_xx];
					double p2 = _temp_y[_mid_yy][_mid_xx];
					double p3 = _temp_z[_mid_yy][_mid_xx];

					XnPoint3D ppp[1]; 
					XnPoint3D ppp_out[1]; 
					ppp[0].X = _mid_x;
					ppp[0].Y = _mid_y;
					ppp[0].Z = _mid_z;


					//_depth.ConvertRealWorldToProjective(1,ppp,ppp_out);

					//cv::Mat color_temp; color2.copyTo(color_temp,leg_temp);

					//char buff1[100];
					//sprintf(buff1," real: %.2f , %.2f , %.2f", ppp[0].X , ppp[0].Y , ppp[0].Z);
					////char buff2[100];
					////sprintf(buff2," proj: %.2f , %.2f , %.2f", ppp_out[0].X , ppp_out[0].Y , ppp_out[0].Z);
					//char buff3[100];
					//sprintf(buff3," med : %.2f , %.2f , %.2f", p1 , p2 , p3);

					//cv::putText(color_temp, buff1, cvPoint(30 , 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 1., cvScalar(255,255,255), 1, CV_AA);
					//cv::putText(color_temp, buff3, cvPoint(30 , 60), cv::FONT_HERSHEY_COMPLEX_SMALL, 1., cvScalar(255,255,255), 1, CV_AA);
					////cv::putText(color_temp, buff2, cvPoint(30 , 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 1., cvScalar(255,255,255), 1, CV_AA);

					//cv::circle(color_temp,cv::Point(ppp_out[0].X, XN_VGA_Y_RES - ppp_out[0].Y),5,cv::Scalar(255,255,255),-1);
					//cv::circle(color_temp,cv::Point(_mid_xx, _mid_yy),5,cv::Scalar(255,255,255),-1);
					//cv::imshow("Leg",color_temp);

					int mxx = (int)((ppp[0].X - _min_xx)/scale);
					int myy = (int)((ppp[0].Z + 30 - _min_yy)/scale);	
					if(mxx >= 0 && myy >= 0){
						//MPcv::circle(top_countours,cv::Point(mxx, myy),5,cv::Scalar(50,50,50),-1);
						//MPcv::circle(top_countours,cv::Point(mxx, myy),2,cv::Scalar(255,255,255),-1);
					}
					XnPoint3D ptleg2; ptleg2.X = _mid_x; ptleg2.Y = _mid_y; ptleg2.Z = _mid_z;

					float leg_dist = distanceToPlane(ptleg2,a,b,c,d);
					//char buff4[100];
					//sprintf(buff4," Dist to Plane: %.2f", leg_dist);
					//cv::putText(top, buff4, cvPoint(30 , 400), cv::FONT_HERSHEY_COMPLEX_SMALL, 1., cvScalar(255,255,255), 1, CV_AA);
					////cv::imshow("top",top);
				
					viewer.addSphere(pcl::PointXYZ(ptleg2.X,ptleg2.Y,ptleg2.Z+50),10,0,0,255,"Corte2");
					//viewer.addSphere(pcl::PointXYZ(_mid_x+(a*-leg_dist),_mid_y+(b*-leg_dist)+50,_mid_z+(c*-leg_dist)+50),10,"Leg2");

										
					//LEG POINT OUT
					_leg_3d_xx = ptleg2.X;
					_leg_3d_yy = ptleg2.Y;
					_leg_3d_zz = ptleg2.Z;
#ifdef _SOCKETS
					char buff1[100];
					message.append("#");
					sprintf_s(buff1,"%.4f %.4f %.4f|",ptleg2.X,ptleg2.Y,ptleg2.Z+50);
					message.append(buff1);
					counter++;
#endif

					double _x_x = ptleg2.X - ptleg.X;
					double _y_y = ptleg2.Y - ptleg.Y;
					double _z_z = ptleg2.Z - ptleg.Z;

					double norm = sqrt(_x_x*_x_x + _y_y*_y_y + _z_z*_z_z);
					_x_x /= norm;
					_y_y /= norm;
					_z_z /= norm;

					//XnPoint3D pt_2;pt_2.X = _x_x;pt_2.Y = _y_y;pt_2.Z = _z_z;

					//leg_dist = distanceToPlane(ptleg2,a,b,c,d);

					viewer.addSphere(pcl::PointXYZ(_mid_x+(_x_x*-leg_dist) + (a* 50),
												   _mid_y+(_y_y*-leg_dist) + (b* 50),
												   _mid_z+50+(_z_z*-leg_dist) + (c* 50)),
												   10,0,0,255,"Calcanhar");

					//HEEL POINT OUT
					_heel_3d_xx = _mid_x+(_x_x*-leg_dist) + (a* 50);
					_heel_3d_yy = _mid_y+(_y_y*-leg_dist) + (b* 50);
					_heel_3d_zz = _mid_z+(_z_z*-leg_dist) + (c* 50) + 50;
#ifdef _SOCKETS
					char buff2[100];
					sprintf_s(buff2,"%.4f %.4f %.4f|",_mid_x+(_x_x*-leg_dist) + (a* 50),
													  _mid_y+(_y_y*-leg_dist) + (b* 50),
													  _mid_z+(_z_z*-leg_dist) + (c* 50) + 50);
					message.append(buff2);
					counter++;
#endif

			//PONTA DO PE
			for(unsigned int i = 0; i< contours.size(); i++ ){
				if(contours[i].size() > 75){
					//cv::Scalar clr = cv::Scalar(255);
					//for(unsigned int j = 0 ; j < contours[i].size() ; j++){
					//	cv::circle(top_view_color,contours[i][j],1,clr,-1);
					//}

					cv::Scalar clr = cv::Scalar(255);// rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
					cv::Rect rect = cv::boundingRect(contours[i]);
					drawContours( top_countours, contours, i, clr, -1, 1, hierarchy, 0, cv::Point() );
					//

					int xxx = 0;
					int yyy = 0;
					int acc = 0;

					if(rect.width / rect.height < 0.75 ){
						cv::putText(top_countours, "Front", cvPoint(30 , 400), cv::FONT_HERSHEY_COMPLEX_SMALL, 1., cvScalar(255,255,255), 1, CV_AA);

						int max_rows = rect.height *0.2;
						for(int yy = rect.y ; yy < rect.y + max_rows ; yy++){ //Lines
							for(int xx = rect.x ; xx < rect.x + rect.width ; xx++){ //Cols
								if(ptr_cont[yy * _width + xx]){
									xxx += xx;
									yyy += yy;
									acc ++;
								}
							}
						}
					}
					else{
						if(rect.width / rect.height > 1.25){
							cv::putText(top_countours, "Side", cvPoint(30 , 400), cv::FONT_HERSHEY_COMPLEX_SMALL, 1., cvScalar(255,255,255), 1, CV_AA);
							//cv::Point(mxx, myy)

							if(mxx-rect.x > rect.width /2.0){
								int max_lines = rect.width *0.2;
								for(int yy = rect.y ; yy < rect.y + rect.height ; yy++){ //Lines
									for(int xx = rect.x ; xx < rect.x + max_lines ; xx++){ //Cols
										if(ptr_cont[yy * _width + xx]){
											xxx += xx;
											yyy += yy;
											acc ++;
										}
									}
								}
							}
							else{
								int max_lines = rect.width *0.2;
								for(int yy = rect.y ; yy < rect.y + rect.height ; yy++){ //Lines
									for(int xx = rect.x + rect.width - max_lines ; xx < rect.x + rect.width ; xx++){ //Cols
										if(ptr_cont[yy * _width + xx]){
											xxx += xx;
											yyy += yy;
											acc ++;
										}
									}
								}
							}
						}
						else{
							cv::putText(top_countours, "middle", cvPoint(30 , 400), cv::FONT_HERSHEY_COMPLEX_SMALL, 1., cvScalar(255,255,255), 1, CV_AA);
							if(mxx-rect.x > rect.width /2.0){
								int max_rows = rect.height *0.3;
								int max_lines = rect.width *0.3;
								for(int yy = rect.y ; yy < rect.y + max_rows ; yy++){ //Lines
									for(int xx = rect.x ; xx < rect.x + max_lines ; xx++){ //Cols
										if(ptr_cont[yy * _width + xx]){
											xxx += xx;
											yyy += yy;
											acc ++;
										}
									}
								}
							}
							else{
								int max_rows = rect.height *0.3;
								int max_lines = rect.width *0.3;
								for(int yy = rect.y ; yy < rect.y + max_rows ; yy++){ //Lines
									for(int xx = rect.x + rect.width - max_lines; xx < rect.x + rect.width ; xx++){ //Cols
										if(ptr_cont[yy * _width + xx]){
											xxx += xx;
											yyy += yy;
											acc ++;
										}
										ptr_cont[yy * _width + xx] = 128;
									}
								}
							}
						}
					}
					if(acc){
						xxx /= acc;
						yyy /= acc;
						if(xxx >= 0 && yyy >= 0 && xxx < 490 && yyy < 490){
							//MPcv::circle(top_countours,cv::Point(xxx,yyy),5,cv::Scalar(20),-1);
							//MPcv::circle(top_countours,cv::Point(xxx,yyy),2,cv::Scalar(255),-1);
						
							int control = 0;
							while(_back_positions[yyy][xxx][1] == 0 && xxx < 490 && yyy < 490){
								if(control % 2){
									xxx++;
								}
								else{
									yyy++;
								}
								control++;
							}
							foot_point = realWorld[_back_positions[yyy][xxx][1] * XN_VGA_X_RES + _back_positions[yyy][xxx][0]];
							viewer.addSphere(pcl::PointXYZ(foot_point.X,foot_point.Y-20,foot_point.Z),10,0,0,255,"PontaDoPe");

							//TIPTOE POINT OUT
							_tiptoe_3d_xx = foot_point.X;
							_tiptoe_3d_yy = foot_point.Y;
							_tiptoe_3d_zz = foot_point.Z;

#ifdef _SOCKETS					
							char buff3[100];
							sprintf_s(buff3,"%.4f %.4f %.4f\n",foot_point.X,foot_point.Y-20,foot_point.Z);
							message.append(buff3);
							counter++;
#endif
						}
					}

					cv::rectangle(top_countours,rect, clr);

				}
			}

					viewer.addLine(	pcl::PointXYZ(ptleg2.X,ptleg2.Y,ptleg2.Z+50),
									pcl::PointXYZ(_mid_x+(_x_x*-leg_dist) + (a* 50),
												  _mid_y+(_y_y*-leg_dist) + (b* 50),
												  _mid_z+50+(_z_z*-leg_dist) + (c* 50)),
						            255,0,0,"line_leg");

					viewer.addLine(	pcl::PointXYZ(_mid_x+(_x_x*-leg_dist) + (a* 50),
												  _mid_y+(_y_y*-leg_dist) + (b* 50),
												  _mid_z+50+(_z_z*-leg_dist) + (c* 50)),
									pcl::PointXYZ(foot_point.X,foot_point.Y-20,foot_point.Z),
									255,0,0,"line_foot");
				}
			//} //END LEG

			//MPcv::imshow("top",top);
			//MPcv::imshow("top_w",top_w);
			//MPcv::imshow("top_contours",top_countours);
		}
		else{
			XnPoint3D point2;

			uchar* ptr = mask_cv.data;
			for(int y=0; y<XN_VGA_Y_RES; y+=2) { 
				for(int x=0; x<XN_VGA_X_RES; x+=2) { 
					XnPoint3D point1;
					point1.X = x; 
					point1.Y = y; 
					point1.Z = _depthMD[y * XN_VGA_X_RES + x]; 

					pointList[y * XN_VGA_X_RES + x] = point1;
				}
			} 

			_depth.ConvertProjectiveToRealWorld(XN_VGA_Y_RES*XN_VGA_X_RES, pointList, realWorld); 

			for(int y=0; y<XN_VGA_Y_RES; y+=2) { 
				for(int x=0; x<XN_VGA_X_RES; x+=2) { 
					cloud.push_back(pcl::PointXYZ(realWorld[y * XN_VGA_X_RES + x].X,realWorld[y * XN_VGA_X_RES + x].Y,realWorld[y * XN_VGA_X_RES + x].Z));
				}
			}
		}
		
#ifdef _SOCKETS
		try{
			if(counter == 3){
				size_t request_length = message.size();

				puts(message.data());

				boost::asio::write(s, boost::asio::buffer(message.data(), request_length));
			}
		} 
		catch(std::exception& e)
		{
			std::cerr << "Exception: " << e.what() << "\n";
		}
#endif

		viewer.addPointCloud(cloud.makeShared(),"cloud");
		viewer.spinOnce (100);
		cv::Mat3b cor(480,640);
		double min = _min_bar,max = _max_bar;
		compute_color_encoded_depth(depthMat16UC1,cor,&min,&max);

		cor.copyTo(color3,mask_cv);

		cv::imshow("Ranged Image",color3);
		//cv::imshow("Depth Image",depthMat8UC1);
		//cv::imshow("Color Image",color2);
		
		//cv::imshow("color",);

		++_frame_counter;
		if (_frame_counter == 10)
		{
			double current_tick = cv::getTickCount();
			_frame_rate = _frame_counter / ((current_tick - _last_tick)/cv::getTickFrequency());
			_last_tick = current_tick;
			_frame_counter = 0;
			printf("%.2f\n",_frame_rate);
		}
	}
}