#include "DepthSensorPlayback.h"

#include <stdio.h>

using namespace std;

#include <Macros\DepthSensorMacros.cs>
#include <DepthSensorDLL.h>
#include <RealSense.h>

#include <opencv2\opencv.hpp>
#include <time.h>

string depthWindowNameRealsensePlayback[] = { "Depth0", "Depht1", "Depht2", "Depht3" };
string maskWindowNameRealsensePlayback[] = { "Mask0", "Mask1", "Mask2", "Mask3" };
string colorWindowNameRealsensePlayback[] = { "Color0", "Color1", "Color2", "Color3" };

void CreateWindows_RealsenseKinectsPlayback(int nKinects) {
	int windowDeltaY = -1200;
	int windowDeltaX = 960;
	int windowSpaceWidth = 640;
	int windowSpaceHeight = 480;

	for (int i = 0; i < nKinects; ++i) {
		cv::namedWindow(depthWindowNameRealsensePlayback[i]);
		cv::moveWindow(depthWindowNameRealsensePlayback[i], windowDeltaX + windowSpaceWidth * i, 0 + windowDeltaY);

		cv::namedWindow(maskWindowNameRealsensePlayback[i]);
		cv::moveWindow(maskWindowNameRealsensePlayback[i], windowDeltaX + windowSpaceWidth * i, windowSpaceHeight + windowDeltaY);

		cv::namedWindow(colorWindowNameRealsensePlayback[i]);
		cv::moveWindow(colorWindowNameRealsensePlayback[i], windowSpaceWidth * i, 0);
	}
}

int main_DepthSensorPlayback(int argc, char* argv[]) {
	std::cout << "main_DepthSensorPlayback" << std::endl;
	std::cout << SensorVersion() << std::endl;

	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

	int depthWidth = 512;
	int depthHeight = 424;
	int colorWidth = 1920;
	int colorHeight = 1080;

	int deviceController = DEVICES::REALSENSE;

	int nDevices = 2;//GetDeviceCounter(deviceController);
	std::cout << "N Devices: " << nDevices << std::endl;

	CreateWindows_RealsenseKinectsPlayback(nDevices);

	uchar** buffDepth = (uchar**)malloc(sizeof(uchar*) * nDevices);
	cv::Mat** depthImage = (cv::Mat**)malloc(sizeof(cv::Mat*) * nDevices);

	uchar** buffDepthMask = (uchar**)malloc(sizeof(uchar*) * nDevices);
	cv::Mat** depthImageMask = (cv::Mat**)malloc(sizeof(cv::Mat*) * nDevices);

	uchar** buffColor = (uchar**)malloc(sizeof(uchar*) * nDevices);
	cv::Mat** colorImage = (cv::Mat**)malloc(sizeof(cv::Mat*) * nDevices);

	rs2::playback** player = (rs2::playback**)malloc(sizeof(rs2::playback*) * nDevices);;

	for (int i = 0; i < nDevices; ++i) {
		//if (i == 0) {
		int initSensor = InitializeSensor(deviceController, i);

		GetImageSize(SOURCES::DEPTH_RAW, &depthWidth, &depthHeight, 0, i);
		int returnSourceDepth = InitializeSource(SOURCES::DEPTH, depthWidth, depthHeight, i);

		//GetImageSize(SOURCES::COLOR, &colorWidth, &colorHeight, 0, i);
		//int returnSourceColor = InitializeSource(SOURCES::COLOR, colorWidth, colorHeight, i);

		//int setCalibration = SetSensorCalibration("C:\\Prozis\\Data\\System\\Conf_HighDensity.json", i);
		char buff[256];
		sprintf_s(buff, "C:\\Dev\\Experiments\\Realsense-Exp\\Realsense-Exp\\m_file_%d.bag", i);
		RealSense* sensor = (RealSense*)GetDepthSensorInstance(i);
		sensor->conf->enable_device_from_file(buff);

		buffDepth[i] = (uchar*)malloc(sizeof(uchar) * depthWidth * depthHeight * 1);
		depthImage[i] = new cv::Mat(depthHeight, depthWidth, CV_8UC1);

		buffDepthMask[i] = (uchar*)malloc(sizeof(uchar) * depthWidth * depthHeight * 1);
		depthImageMask[i] = new cv::Mat(depthHeight, depthWidth, CV_8UC1);

		player[i] = NULL;
		//buffColor[i] = (uchar*)malloc(sizeof(uchar) * colorWidth * colorHeight * 3);
		//colorImage[i] = new cv::Mat(colorHeight, colorWidth, CV_8UC3);
	}

	for (int i = 0; i < nDevices; ++i) {
		int startSensor = StartSensor(i);
	}

	while (1) {
		for (int i = 0; i < nDevices; ++i) {
			UpdateSensor(i);
			//UpdateSource(SOURCES::DEPTH, i);
			//UpdateSource(SOURCES::COLOR, i);

			if (GetImage(SOURCES::DEPTH, buffDepth[i], i) == 0) {
				depthImage[i]->data = buffDepth[i];
				cv::imshow(depthWindowNameRealsensePlayback[i], *depthImage[i]);
			}

			if (GetImage(SOURCES::DEPTH_MASK, buffDepthMask[i], i) == 0) {
				depthImageMask[i]->data = buffDepthMask[i];
				cv::imshow(maskWindowNameRealsensePlayback[i], *depthImageMask[i]);
			}

			//if (GetImage(SOURCES::COLOR, buffColor[i], i) == 0) {
			//	colorImage[i]->data = buffColor[i];
			//	cv::imshow(colorWindowNameRealsense[i], *colorImage[i]);
			//}
		}

		++_frame_counter;
		if (_frame_counter == 60)
		{
			double current_tick = cv::getTickCount();
			_frame_rate = _frame_counter / ((current_tick - _last_tick) / cv::getTickFrequency());
			_last_tick = current_tick;
			_frame_counter = 0;
			printf("FrameRate %.2f\n", _frame_rate);
		}

		char c = cv::waitKey(1);

		if (c == 27) {
			break;
		}

		if (c == ' ') {
			//int count = GetDeviceCounter(deviceController);

			for (int i = 0; i < 2; ++i) {
				RealSense* sensor = (RealSense*)GetDepthSensorInstance(i);

				//char buff[16];
				//sprintf_s(buff, "m_file_%d.bag", i);
				//recorder[i] = new rs2::recorder(buff, sensor->pipe->get_active_profile().get_device());

			}

			printf("Pause/Play Recording");
		}
	}

	for (int i = 0; i < nDevices; ++i) {
		if (player[i] != NULL) {
			delete player[i];
		}

		CloseSensor(i);
	}

	return 0;
}