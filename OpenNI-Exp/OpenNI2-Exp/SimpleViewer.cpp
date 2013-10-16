#include "SimpleViewer.h"

#include <OpenNI.h>
#include <opencv2\opencv.hpp>

int main_simple_viewer(int argc, char* argv[]){
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

	if (!depth.isValid() || !color.isValid())
	{
		printf("SimpleViewer: No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return 2;
	}

	openni::VideoMode depthVideoMode;
	openni::VideoMode colorVideoMode;
	openni::VideoStream& m_depthStream(depth);
	openni::VideoStream& m_colorStream(color);

	depthVideoMode = m_depthStream.getVideoMode();
	//openni::VideoMode mode;
	//mode.setResolution(640,480);
	//mode.setFps(30);

	//rc = m_depthStream.setVideoMode(mode);
	colorVideoMode = m_colorStream.getVideoMode();

	int depthWidth = depthVideoMode.getResolutionX();
	int depthHeight = depthVideoMode.getResolutionY();
	int colorWidth = colorVideoMode.getResolutionX();
	int colorHeight = colorVideoMode.getResolutionY();

	openni::VideoFrameRef		m_depthFrame;
	openni::VideoFrameRef		m_colorFrame;

	openni::VideoStream**		m_streams = new openni::VideoStream*[2];
	m_streams[0] = &m_depthStream;
	m_streams[1] = &m_colorStream;
		
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

		cv::Mat depthMat16UC1(480,640,CV_16UC1,(void*)m_depthFrame.getData());
		cv::Mat depthMat8UC1;
		depthMat16UC1.convertTo(depthMat8UC1, CV_8UC1,0.05);

		m_colorStream.readFrame(&m_colorFrame);

		cv::Mat color(480,640,CV_8UC3,(void*) m_colorFrame.getData());
		cv::Mat color2;
		cv::cvtColor(color,color2,CV_RGB2BGR);

		cv::imshow("color",color2);
		cv::imshow("Depth",depthMat8UC1);
	}

	m_depthStream.stop();
	m_colorStream.stop();
	m_depthStream.destroy();
	m_colorStream.destroy();
	device.close();
	openni::OpenNI::shutdown();

	return 0;
}