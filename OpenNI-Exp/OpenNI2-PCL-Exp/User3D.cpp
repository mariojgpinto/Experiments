#include "User3D.h"

#include <OpenNI.h>
#include <NiTE.h>
#include <opencv2\opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

int main_user_3d(int argc, char* argv[]){
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

	rc = device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );
	rc = device.setDepthColorSyncEnabled(true);

	openni::VideoFrameRef		m_depthFrame;
	openni::VideoFrameRef		m_colorFrame;

	openni::VideoStream& m_depthStream(depth);
	openni::VideoStream& m_colorStream(color);

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

	pcl::visualization::CloudViewer viewer("Simple cloud_file Viewer");
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	cloud.width = 640*480;
	cloud.height = 1;
	cloud.points.resize (cloud.width * cloud.height);

	char c = 0;
	while((c = cv::waitKey(12)) != 27){
		int changedIndex;
		openni::Status rc = openni::OpenNI::waitForAnyStream(m_streams, 2, &changedIndex);
		if (rc != openni::STATUS_OK)
		{
			printf("Wait failed\n");
			return 3;
		}

		m_depthStream.readFrame(&m_depthFrame);

		nite::Status rc_nite = m_pUserTracker->readFrame(&userTrackerFrame);
		if (rc_nite != nite::STATUS_OK)
		{
			printf("GetNextData failed\n");
			return 3;
		}

		const nite::UserMap& userLabels = userTrackerFrame.getUserMap();
		

		//user_map.

		cv::Mat depthMat16UC1(480,640,CV_16UC1,(void*)m_depthFrame.getData());
		cv::Mat depthMat8UC1;
		depthMat16UC1.convertTo(depthMat8UC1, CV_8UC1,0.05);

		cv::Mat depthMat16UC1_nite(480,640,CV_16UC1,(void*)userLabels.getPixels());
		cv::Mat depthMat8UC1_nite;
		depthMat16UC1_nite.convertTo(depthMat8UC1_nite, CV_8UC1);

		cv::Mat temp;

		depthMat8UC1.copyTo(temp,depthMat8UC1_nite);




		rc = m_colorStream.readFrame(&m_colorFrame);

		cv::Mat color(480,640,CV_8UC3,(void*) m_colorFrame.getData());
		cv::Mat color2;
		cv::cvtColor(color,color2,CV_RGB2BGR);
		cv::Mat color3;
		color2.copyTo(color3,depthMat8UC1_nite);

		cv::imshow("Depth",depthMat8UC1);
		cv::imshow("Depth_nite",color3);

		
		//****************************************************************
		//* 3D
		//****************************************************************
		cloud.points.clear();
		
		uchar* ptr_mask = (uchar*)depthMat8UC1_nite.data;
		uint16_t* ptr = (uint16_t*)m_depthFrame.getData();
		openni::RGB888Pixel* ptr_clr = (openni::RGB888Pixel*)m_colorFrame.getData();
		
		bool new_points = false;
		for(int y=0; y<480; y++) { 
			for(int x=0; x<640; x++) {
				float d_x = x;
				float d_y = y;
				float d_z =  ptr[y * 640 + x];
				float w_x = 0;
				float w_y = 0;
				float w_z = 0;

				if(d_z > 0 && ptr_mask[y * 640 + x]){
					openni::CoordinateConverter::convertDepthToWorld(m_depthStream,d_x,d_y,d_z,&w_x,&w_y,&w_z);

					pcl::PointXYZRGB pt(ptr_clr[y * 640 + x].r,
										ptr_clr[y * 640 + x].g,
										ptr_clr[y * 640 + x].b);
					pt.x = w_x;
					pt.y = w_y;
					pt.z = w_z;

					cloud.push_back(pcl::PointXYZRGB(pt));

					new_points = true;
				}
			}
		} 

		if(new_points)
			viewer.showCloud(cloud.makeShared());



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

	exit (0);

	/*m_depthStream.stop();
	m_depthStream.destroy();
		
	delete m_pUserTracker;

	device.close();
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();*/

	return 0;
}