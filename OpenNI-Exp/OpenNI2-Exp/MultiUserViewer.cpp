#include "MultiUserViewer.h"

#include <OpenNI.h>
#include <NiTE.h>
#include <opencv2\opencv.hpp>

int main_multi_user_viewer(int argc, char* argv[]){
	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

	openni::Status rc = openni::STATUS_OK;

	openni::Device device1, device2;
	openni::VideoStream depth1, depth2;

	rc = openni::OpenNI::initialize();
	if (rc != openni::STATUS_OK)
	{
		printf("%s: Initialize failed\n%s\n", argv[0], openni::OpenNI::getExtendedError());
		return 1;
	}

	openni::Array<openni::DeviceInfo> deviceList;
	openni::OpenNI::enumerateDevices(&deviceList);

	const char* device1Uri;
	const char* device2Uri;

	if (deviceList.getSize() < 2){
		printf("Missing devices\n");
		openni::OpenNI::shutdown();
		return 1;
	}
	device1Uri = deviceList[1].getUri();
	device2Uri = deviceList[0].getUri();

	rc = device1.open(device1Uri);
	if (rc != openni::STATUS_OK)
	{
		printf("%s: Couldn't open device %s\n%s\n", argv[0], device1Uri, openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 3;
	}

	rc = device2.open(device2Uri);
	if (rc != openni::STATUS_OK)
	{
		printf("%s: Couldn't open device %s\n%s\n", argv[0], device2Uri, openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 3;
	}

	rc = depth1.create(device1, openni::SENSOR_DEPTH);
	if (rc != openni::STATUS_OK)
	{
		printf("%s: Couldn't create stream %d on device %s\n%s\n", argv[0], openni::SENSOR_DEPTH, device1Uri, openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 4;
	}
	rc = depth2.create(device2, openni::SENSOR_DEPTH);
	if (rc != openni::STATUS_OK)
	{
		printf("%s: Couldn't create stream %d on device %s\n%s\n", argv[0], openni::SENSOR_DEPTH, device2Uri, openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 4;
	}

	rc = depth1.start();
	if (rc != openni::STATUS_OK)
	{
		printf("%s: Couldn't start stream %d on device %s\n%s\n", argv[0], openni::SENSOR_DEPTH, device1Uri, openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 5;
	}
	rc = depth2.start();
	if (rc != openni::STATUS_OK)
	{
		printf("%s: Couldn't start stream %d on device %s\n%s\n", argv[0], openni::SENSOR_DEPTH, device2Uri, openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 5;
	}

	if (!depth1.isValid() && !depth2.isValid())
	{
		printf("SimpleViewer: No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return 6;
	}

	openni::VideoStream& m_depthStream1(depth1);
	openni::VideoStream& m_depthStream2(depth2);

	openni::VideoFrameRef		m_depthFrame1;
	openni::VideoFrameRef		m_depthFrame2;

	openni::VideoStream**		m_streams = new openni::VideoStream*[2];
	m_streams[0] = &m_depthStream1;
	m_streams[1] = &m_depthStream2;

	nite::NiTE::initialize();
	nite::UserTracker* m_pUserTracker1 = new nite::UserTracker;
	nite::UserTracker* m_pUserTracker2 = new nite::UserTracker;

	if (m_pUserTracker1->create(&device1) != nite::STATUS_OK)	{
		return openni::STATUS_ERROR;
	}

	if (m_pUserTracker2->create(&device2) != nite::STATUS_OK)	{
		return openni::STATUS_ERROR;
	}

	nite::UserTrackerFrameRef userTrackerFrame1;
	nite::UserTrackerFrameRef userTrackerFrame2;

	char c = 0;
	while((c = cv::waitKey(12)) != 27){
		int changedIndex;
		openni::Status rc = openni::OpenNI::waitForAnyStream(m_streams, 2, &changedIndex);
		if (rc != openni::STATUS_OK)
		{
			printf("Wait failed\n");
			return 3;
		}

		m_depthStream1.readFrame(&m_depthFrame1);
		m_depthStream2.readFrame(&m_depthFrame2);

		cv::Mat depthMat16UC1_1(480,640,CV_16UC1,(void*)m_depthFrame1.getData());
		cv::Mat depthMat8UC1_1;
		depthMat16UC1_1.convertTo(depthMat8UC1_1, CV_8UC1,0.05);

		cv::Mat depthMat16UC1_2(480,640,CV_16UC1,(void*)m_depthFrame2.getData());
		cv::Mat depthMat8UC1_2;
		depthMat16UC1_2.convertTo(depthMat8UC1_2, CV_8UC1,0.05);





		nite::Status rc_nite = m_pUserTracker1->readFrame(&userTrackerFrame1);
		if (rc_nite != nite::STATUS_OK)
		{
			printf("GetNextData failed\n");
			return 4;
		}
		rc_nite = m_pUserTracker2->readFrame(&userTrackerFrame2);
		if (rc_nite != nite::STATUS_OK)
		{
			printf("GetNextData failed\n");
			return 4;
		}

		const nite::UserMap& userLabels1 = userTrackerFrame1.getUserMap();
		const nite::UserMap& userLabels2 = userTrackerFrame2.getUserMap();

		cv::Mat depthMat16UC1_nite1(480,640,CV_16UC1,(void*)userLabels1.getPixels());
		cv::Mat depthMat8UC1_nite1;
		depthMat16UC1_nite1.convertTo(depthMat8UC1_nite1, CV_8UC1);

		cv::Mat depthMat16UC1_nite2(480,640,CV_16UC1,(void*)userLabels2.getPixels());
		cv::Mat depthMat8UC1_nite2;
		depthMat16UC1_nite2.convertTo(depthMat8UC1_nite2, CV_8UC1);

		cv::Mat temp1,temp2;
		depthMat8UC1_1.copyTo(temp1,depthMat8UC1_nite1);
		depthMat8UC1_2.copyTo(temp2,depthMat8UC1_nite2);

		cv::imshow("Depth_1",temp1);
		cv::imshow("Depth_2",temp2);

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

	m_depthStream1.stop();
	m_depthStream2.stop();
	m_depthStream1.destroy();
	m_depthStream2.destroy();
	device1.close();
	//device2.close();
	openni::OpenNI::shutdown();

	return 0;
}