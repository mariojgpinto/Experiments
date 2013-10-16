#include "SimpleUserViewer.h"

#include <OpenNI.h>
#include <NiTE.h>
#include <opencv2\opencv.hpp>

int main_simple_user_viewer(int argc, char* argv[]){
	openni::Status rc = openni::STATUS_OK;

	openni::Device device;
	openni::VideoStream depth;

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

	openni::VideoMode depthVideoMode;
	openni::VideoStream& m_depthStream(depth);

	depthVideoMode = m_depthStream.getVideoMode();

	openni::VideoFrameRef		m_depthFrame;

	openni::VideoStream**		m_streams = new openni::VideoStream*[1];
	m_streams[0] = &m_depthStream;





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

	char c = 0;
	while((c = cv::waitKey(12)) != 27){
		int changedIndex;
		openni::Status rc = openni::OpenNI::waitForAnyStream(m_streams, 1, &changedIndex);
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


		cv::imshow("Depth",depthMat8UC1);
		cv::imshow("Depth_nite",temp);
	}

	m_depthStream.stop();
	m_depthStream.destroy();
		
	delete m_pUserTracker;

	device.close();
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();

	return 0;
}