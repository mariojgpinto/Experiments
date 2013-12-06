#include "SimpleUserViewer.h"

#include <OpenNI.h>
#include <NiTE.h>
#include <opencv2\opencv.hpp>

int main_simple_user_viewer(int argc, char* argv[]){
	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

	openni::Status rc = openni::STATUS_OK;

	openni::Device device;
	openni::VideoStream depth, color;
	const char* deviceURI = openni::ANY_DEVICE;//"Track\\track1.oni";//
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
		else{
			depth.setMirroringEnabled(false);
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
		else{
			//color.setMirroringEnabled(false);
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





	nite::Status et = nite::NiTE::initialize();
	nite::UserTracker* m_pUserTracker = new nite::UserTracker();
	
	if (m_pUserTracker->create(&device) != nite::STATUS_OK)	{
		return openni::STATUS_ERROR;
	}

	nite::SkeletonState g_skeletonStates[10] = {nite::SKELETON_NONE};

	nite::UserTrackerFrameRef userTrackerFrame;
	openni::VideoFrameRef depthFrame;
	openni::VideoFrameRef depthFrame_nite;
	
	nite::UserMap user_map;

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

		double min, max;
		cv::minMaxIdx(depthMat8UC1_nite,&min,&max);


		rc = m_colorStream.readFrame(&m_colorFrame);

		cv::Mat color(480,640,CV_8UC3,(void*) m_colorFrame.getData());
		cv::Mat color2;
		cv::cvtColor(color,color2,CV_RGB2BGR);
		cv::Mat color3;
		color2.copyTo(color3,depthMat8UC1_nite);


		


		const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();

		if(users.getSize()){
			for(int i = 0 ; i < users.getSize() ; ++i){
				int id = users[i].getId();
				const nite::BoundingBox bbox = users[i].getBoundingBox();
				const nite::Point3f cm = users[i].getCenterOfMass();
				printf("");

				float px,py,pz;
				openni::CoordinateConverter::convertWorldToDepth(depth,cm.x,cm.y,cm.z,&px,&py,&pz);

				cv::rectangle(color3,cv::Rect(bbox.min.x,bbox.min.y,bbox.max.x - bbox.min.x, bbox.max.y - bbox.min.y),
									 cv::Scalar(255,0,0),3);

				cv::circle(color3, cv::Point(px,py), 5, cv::Scalar(0,0,255), -1);
			}
		}

		cv::imshow("Depth",depthMat8UC1);
		cv::imshow("Depth_nite",depthMat8UC1_nite);
		cv::imshow("User_mask",color3);

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

	exit(0);

	m_depthStream.stop();
	m_depthStream.destroy();
		
	delete m_pUserTracker;

	device.close();
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();

	return 0;
}