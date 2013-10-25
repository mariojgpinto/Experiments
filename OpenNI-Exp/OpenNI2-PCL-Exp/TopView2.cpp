#include "TopView2.h"

#include <OpenNI.h>
#include <NiTE.h>
#include <opencv2\opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

float distanceToPlane(float pt_x, float pt_y, float pt_z, float a, float b, float c, float d){
	float v = a*pt_x + b*pt_y + c*pt_z + d;
	v /= sqrt(a*a+b*b+c*c);
	return std::abs(v);
}

int main_top_view_2(int argc, char* argv[]){
	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

	openni::Status rc = openni::STATUS_OK;

	openni::Device device;
	openni::VideoStream depth, color;
	const char* deviceURI = openni::ANY_DEVICE;
	if (argc > 1)
	{
		deviceURI = argv[1];
	}

	rc = openni::OpenNI::initialize();

	printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());
	
	rc = device.open(deviceURI);
	
	
	if (rc != openni::STATUS_OK)
	{
		printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		getchar();
		openni::OpenNI::shutdown();
		return 1;
	}

	rc = depth.create(device, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		rc = depth.start();
		if (rc != openni::STATUS_OK)
		{
			printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			depth.destroy();
		}
	}
	else
	{
		printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
	}

	rc = color.create(device, openni::SENSOR_COLOR);
	if (rc == openni::STATUS_OK)
	{
		rc = color.start();
		if (rc != openni::STATUS_OK)
		{
			printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
			color.destroy();
		}
	}
	else
	{
		printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
	}

	if (!depth.isValid() || !color.isValid())
	{
		printf("SimpleViewer: No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return 2;
	}

	rc = device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );
	rc = device.setDepthColorSyncEnabled(true);

	openni::VideoMode depthVideoMode;
	openni::VideoMode colorVideoMode;
	openni::VideoStream& m_depthStream(depth);
	openni::VideoStream& m_colorStream(color);

	openni::VideoFrameRef		m_depthFrame;
	openni::VideoFrameRef		m_colorFrame;
		
	openni::VideoStream**		m_streams = new openni::VideoStream*[2];
	m_streams[0] = &m_depthStream;
	m_streams[1] = &m_colorStream;
		

	nite::NiTE::initialize();
	nite::UserTracker* m_pUserTracker = new nite::UserTracker;

	if (m_pUserTracker->create(&device) != nite::STATUS_OK)	{
		return openni::STATUS_ERROR;
	}

	nite::SkeletonState g_skeletonStates[10] = {nite::SKELETON_NONE};

	nite::UserTrackerFrameRef userTrackerFrame;
	openni::VideoFrameRef depthFrame;
	openni::VideoFrameRef depthFrame_nite;
	
	nite::UserMap user_map;

	bool floor_plane = false;
	nite::Plane plane;
	float plane_a,plane_b,plane_c,plane_d;

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



	pcl::visualization::CloudViewer viewer("Simple cloud_file Viewer");
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	cloud.width = 640*480;
	cloud.height = 1;
	cloud.points.resize (cloud.width * cloud.height);



	char c = 0;
	while((c = cv::waitKey(12)) != 27){
		//****************************************************************
		//* UPDATE OPENNI
		//****************************************************************
		int changedIndex;
		openni::Status rc = openni::OpenNI::waitForAnyStream(m_streams, 2, &changedIndex);
		if (rc != openni::STATUS_OK)
		{
			printf("Wait failed\n");
			return 3;
		}

		nite::Status rc_nite = m_pUserTracker->readFrame(&userTrackerFrame);
		if (rc_nite != nite::STATUS_OK)
		{
			printf("GetNextData failed\n");
			return 3;
		}

		const nite::UserMap& userLabels = userTrackerFrame.getUserMap();

		rc = m_depthStream.readFrame(&m_depthFrame);

		cv::Mat depthMat16UC1(480,640,CV_16UC1,(void*)m_depthFrame.getData());
		cv::Mat depthMat8UC1;
		depthMat16UC1.convertTo(depthMat8UC1, CV_8UC1,0.05);

		rc = m_colorStream.readFrame(&m_colorFrame);

		cv::Mat color(480,640,CV_8UC3,(void*) m_colorFrame.getData());
		cv::Mat color2;
		cv::cvtColor(color,color2,CV_RGB2BGR);
		cv::Mat color3;
		color2.copyTo(color3,depthMat8UC1);

		cv::imshow("Depth",depthMat8UC1);
		cv::imshow("color",color3);

		//****************************************************************
		//* 3D
		//****************************************************************
		cloud.points.clear();
		
		uint16_t* ptr = (uint16_t*)m_depthFrame.getData();
		openni::RGB888Pixel* ptr_clr = (openni::RGB888Pixel*)m_colorFrame.getData();
		
		if(floor_plane){
			for(int y=0; y<480; y++) { 
				for(int x=0; x<640; x++) {
					float d_x = x;
					float d_y = y;
					float d_z =  ptr[y * 640 + x];
					float w_x = 0;
					float w_y = 0;
					float w_z = 0;

					if(d_z > 0){
						openni::CoordinateConverter::convertDepthToWorld(m_depthStream,d_x,d_y,d_z,&w_x,&w_y,&w_z);

						float dist = distanceToPlane(w_x,w_y,w_z,plane_a,plane_b,plane_c,plane_d);
						if(dist > 100){
							pcl::PointXYZRGB pt(ptr_clr[y * 640 + x].r,
												ptr_clr[y * 640 + x].g,
												ptr_clr[y * 640 + x].b);
							pt.x = w_x;
							pt.y = w_y;
							pt.z = w_z;

							cloud.push_back(pcl::PointXYZRGB(pt));


							_temp_x[y][x] = _inverse.row(0).at<double>(0) * w_x + 
											_inverse.row(0).at<double>(1) * w_y +
											_inverse.row(0).at<double>(2) * w_z;

							_temp_y[y][x] = _inverse.row(1).at<double>(0) * w_x + 
											_inverse.row(1).at<double>(1) * w_y +
											_inverse.row(1).at<double>(2) * w_z;

							_temp_z[y][x] = _inverse.row(2).at<double>(0) * w_x + 
											_inverse.row(2).at<double>(1) * w_y +
											_inverse.row(2).at<double>(2) * w_z;

							if(_temp_x[y][x] < _min_xx)
								_min_xx = _temp_x[y][x];
							if(_temp_x[y][x] > _max_xx)
								_max_xx = _temp_x[y][x];
							if(_temp_z[y][x] < _min_yy)
								_min_yy = _temp_z[y][x];
							if(_temp_z[y][x] > _max_yy)
								_max_yy = _temp_z[y][x];

						}
					}
				}
			} 

			_width = _max_xx - _min_xx;
			_height = _max_yy - _min_yy;

			_width /= 10;
			_height /= 10;
			//_width = 500;
			//_height = 500;

			cv::Mat top = cv::Mat::zeros(cv::Size(_width,_height),CV_8UC1);
			uchar* top_ptr = top.data;
			int _x,_y;
			for(int y=0; y<480; y++) { 
				for(int x=0; x<640; x++) { 
					_x = (int)((_temp_x[y][x] - _min_xx)/10.0);
					_y = (int)((_temp_z[y][x] - _min_yy)/10.0);
					if(_y >= 0 && _x >= 0){
						top_ptr[_x * _width + _y] = 255;
					}
				}
			}

			cv::imshow("top",top);
		}
		else{
			for(int y=0; y<480; y++) { 
				for(int x=0; x<640; x++) {
					float d_x = x;
					float d_y = y;
					float d_z =  ptr[y * 640 + x];
					float w_x = 0;
					float w_y = 0;
					float w_z = 0;

					if(d_z > 0){
						openni::CoordinateConverter::convertDepthToWorld(m_depthStream,d_x,d_y,d_z,&w_x,&w_y,&w_z);

						pcl::PointXYZRGB pt(ptr_clr[y * 640 + x].r,
											ptr_clr[y * 640 + x].g,
											ptr_clr[y * 640 + x].b);
						pt.x = w_x;
						pt.y = w_y;
						pt.z = w_z;

						cloud.push_back(pcl::PointXYZRGB(pt));
					}
				}
			} 
		}

		viewer.showCloud(cloud.makeShared());


		//****************************************************************
		//* KEYS
		//****************************************************************
		if(c == 'f'){
			//openni::VideoStream::
			if(floor_plane){
				floor_plane = false;
			}
			else{
				plane = userTrackerFrame.getFloor();
			
				if(plane.normal.x != 0 || plane.normal.y != 0 || plane.normal.z != 0 ){
					plane_a = plane.normal.x;
					plane_b = plane.normal.y;
					plane_c = plane.normal.z;
					plane_d = -(plane.normal.x*plane.point.x + 
								plane.normal.y*plane.point.y + 
								plane.normal.z*plane.point.z);

					cv::Vec3f vec_1(1,0,0);
					cv::Vec3f vec_2 = vec_1.cross(cv::Vec3f(plane_a,plane_b,plane_c));

					_matrix.create(3,3,CV_64FC1);
					_matrix.ptr<double>(0)[0] = vec_1.val[0];
					_matrix.ptr<double>(0)[1] = vec_1.val[1];
					_matrix.ptr<double>(0)[2] = vec_1.val[2];
					_matrix.ptr<double>(1)[0] = -plane_a;
					_matrix.ptr<double>(1)[1] = -plane_b;
					_matrix.ptr<double>(1)[2] = -plane_c;
					_matrix.ptr<double>(2)[0] = vec_2.val[0];
					_matrix.ptr<double>(2)[1] = vec_2.val[1];
					_matrix.ptr<double>(2)[2] = vec_2.val[2];

					_inverse = _matrix.inv(1);

					floor_plane = true;
				}
			}
		}

		//****************************************************************
		//* UPDATE FPS
		//****************************************************************
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

	//m_depthStream.stop();
	//m_colorStream.stop();
	//m_depthStream.destroy();
	//m_colorStream.destroy();
	////device.close();
	//openni::OpenNI::shutdown();
	exit (0);

	return 0;
}