#include "Sample_Body.h"
// This source code is licensed under the MIT license. Please see the License in License.txt.
// "This is preliminary software and/or hardware and APIs are preliminary and subject to change."
//

#include <stdio.h>
#include <tchar.h>

#include <Windows.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>


template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

int main_sample_body(int argc, char* argv[])
{
	cv::setUseOptimized(true);

	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

	// Sensor
	IKinectSensor* pSensor;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor(&pSensor);
	if (FAILED(hResult)){
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
		return -1;
	}

	hResult = pSensor->Open();
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return -1;
	}

	// Source C
	IDepthFrameSource* pDepthSource;
	hResult = pSensor->get_DepthFrameSource(&pDepthSource);
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
		return -1;
	}

	IColorFrameSource* pColorSource;
	hResult = pSensor->get_ColorFrameSource(&pColorSource);
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		return -1;
	}

	IBodyFrameSource* pBodySource;
	hResult = pSensor->get_BodyFrameSource(&pBodySource);
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_BodyFrameSource()" << std::endl;
		return -1;
	}

	// Reader
	IDepthFrameReader* pDepthReader;
	hResult = pDepthSource->OpenReader(&pDepthReader);
	if (FAILED(hResult)){
		std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	IColorFrameReader* pColorReader;
	hResult = pColorSource->OpenReader(&pColorReader);
	if (FAILED(hResult)){
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	IBodyFrameReader* pBodyReader;
	hResult = pBodySource->OpenReader(&pBodyReader);
	if (FAILED(hResult)){
		std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	// Description
	IFrameDescription* pDepthDescription;
	hResult = pDepthSource->get_FrameDescription(&pDepthDescription);
	if (FAILED(hResult)){
		std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}

	IFrameDescription* pColorDescription;
	hResult = pColorSource->get_FrameDescription(&pColorDescription);
	if (FAILED(hResult)){
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}

	int color_width = 0;
	int color_height = 0;
	pColorDescription->get_Width(&color_width); // 1920
	pColorDescription->get_Height(&color_height); // 1080
	unsigned int bufferColorSize = color_width * color_height * 4 * sizeof(unsigned char);

	cv::Mat bufferColorMat(color_height, color_width, CV_8UC4);
	cv::Mat bodyMat(color_height / 2, color_width / 2, CV_8UC4);
	cv::namedWindow("Body");
	cv::namedWindow("Color");



	int depth_width = 0;
	int depth_height = 0;
	pDepthDescription->get_Width(&depth_width); // 512
	pDepthDescription->get_Height(&depth_height); // 424
	unsigned int bufferDepthSize = depth_width * depth_height * sizeof(unsigned short);

	cv::Mat bufferDepthMat(depth_height, depth_width, CV_16SC1);
	cv::Mat depthMat(depth_height, depth_width, CV_8UC1);
	cv::namedWindow("Depth");



	// Color Table
	cv::Vec3b color[6];
	color[0] = cv::Vec3b(255, 0, 0);
	color[1] = cv::Vec3b(0, 255, 0);
	color[2] = cv::Vec3b(0, 0, 255);
	color[3] = cv::Vec3b(255, 255, 0);
	color[4] = cv::Vec3b(255, 0, 255);
	color[5] = cv::Vec3b(0, 255, 255);

	// Coordinate Mapper
	ICoordinateMapper* pCoordinateMapper;
	hResult = pSensor->get_CoordinateMapper(&pCoordinateMapper);
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
		return -1;
	}

	while (1){
		// Frame
		IColorFrame* pColorFrame = nullptr;
		hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
		if (SUCCEEDED(hResult)){
			hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferColorSize, reinterpret_cast<BYTE*>(bufferColorMat.data), ColorImageFormat_Bgra);
			if (SUCCEEDED(hResult)){
				cv::resize(bufferColorMat, bodyMat, cv::Size(), 0.5, 0.5);
			}
		}
		SafeRelease(pColorFrame);

		

		IDepthFrame* pDepthFrame = nullptr;
		hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);
		if (SUCCEEDED(hResult)){
			hResult = pDepthFrame->AccessUnderlyingBuffer(&bufferDepthSize, reinterpret_cast<UINT16**>(&bufferDepthMat.data));
			if (SUCCEEDED(hResult)){
				bufferDepthMat.convertTo(depthMat, CV_8U, -255.0f / 4500.0f, 255.0f);
			}
		}
		SafeRelease(pDepthFrame);

		

		IBodyFrame* pBodyFrame = nullptr;
		hResult = pBodyReader->AcquireLatestFrame(&pBodyFrame);
		if (SUCCEEDED(hResult)){
			IBody* pBody[BODY_COUNT] = { 0 };
			hResult = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);
			if (SUCCEEDED(hResult)){
				for (int count = 0; count < BODY_COUNT; count++){
					BOOLEAN bTracked = false;
					hResult = pBody[count]->get_IsTracked(&bTracked);
					if (SUCCEEDED(hResult) && bTracked){
						Joint joint[JointType::JointType_Count];
						hResult = pBody[count]->GetJoints(JointType::JointType_Count, joint);
						if (SUCCEEDED(hResult)){
							// Left Hand State
							HandState leftHandState = HandState::HandState_Unknown;
							hResult = pBody[count]->get_HandLeftState(&leftHandState);
							if (SUCCEEDED(hResult)){
								ColorSpacePoint colorSpacePoint = { 0 };
								hResult = pCoordinateMapper->MapCameraPointToColorSpace(joint[JointType::JointType_HandLeft].Position, &colorSpacePoint);
								if (SUCCEEDED(hResult)){
									int x = static_cast<int>(colorSpacePoint.X);
									int y = static_cast<int>(colorSpacePoint.Y);
									if ((x >= 0) && (x < color_width) && (y >= 0) && (y < color_height)){
										if (leftHandState == HandState::HandState_Open){
											cv::circle(bufferColorMat, cv::Point(x, y), 75, cv::Scalar(0, 128, 0), 5, CV_AA);
										}
										else if (leftHandState == HandState::HandState_Closed){
											cv::circle(bufferColorMat, cv::Point(x, y), 75, cv::Scalar(0, 0, 128), 5, CV_AA);
										}
										else if (leftHandState == HandState::HandState_Lasso){
											cv::circle(bufferColorMat, cv::Point(x, y), 75, cv::Scalar(128, 128, 0), 5, CV_AA);
										}
									}
								}
							}

							// Right Hand State
							HandState rightHandState = HandState::HandState_Unknown;
							hResult = pBody[count]->get_HandRightState(&rightHandState);
							if (SUCCEEDED(hResult)){
								ColorSpacePoint colorSpacePoint = { 0 };
								hResult = pCoordinateMapper->MapCameraPointToColorSpace(joint[JointType::JointType_HandRight].Position, &colorSpacePoint);
								if (SUCCEEDED(hResult)){
									int x = static_cast<int>(colorSpacePoint.X);
									int y = static_cast<int>(colorSpacePoint.Y);
									if ((x >= 0) && (x < color_width) && (y >= 0) && (y < color_height)){
										if (rightHandState == HandState::HandState_Open){
											cv::circle(bufferColorMat, cv::Point(x, y), 75, cv::Scalar(0, 128, 0), 5, CV_AA);
										}
										else if (rightHandState == HandState::HandState_Closed){
											cv::circle(bufferColorMat, cv::Point(x, y), 75, cv::Scalar(0, 0, 128), 5, CV_AA);
										}
										else if (rightHandState == HandState::HandState_Lasso){
											cv::circle(bufferColorMat, cv::Point(x, y), 75, cv::Scalar(128, 128, 0), 5, CV_AA);
										}
									}
								}
							}

							// Joint
							for (int type = 0; type < JointType::JointType_Count; type++){
								ColorSpacePoint colorSpacePoint = { 0 };
								pCoordinateMapper->MapCameraPointToColorSpace(joint[type].Position, &colorSpacePoint);
								int x = static_cast<int>(colorSpacePoint.X);
								int y = static_cast<int>(colorSpacePoint.Y);
								if ((x >= 0) && (x < color_width) && (y >= 0) && (y < color_height)){
									cv::circle(bufferColorMat, cv::Point(x, y), 5, static_cast< cv::Scalar >(color[count]), -1, CV_AA);
								}

								DepthSpacePoint depthSpacePoint = { 0 };
								pCoordinateMapper->MapCameraPointToDepthSpace(joint[type].Position, &depthSpacePoint);
								int dx = static_cast<int>(depthSpacePoint.X);
								int dy = static_cast<int>(depthSpacePoint.Y);
								if ((dx >= 0) && (dx < depth_width) && (dy >= 0) && (dy < depth_height)){
									//cv::circle(bufferDepthMat, cv::Point(dx, dy), 3, cv::Scalar(255, 255, 255), -1, CV_AA);
								}
							}
						}
					}
				}
				cv::resize(bufferColorMat, bodyMat, cv::Size(), 0.5, 0.5);
			}
		}
		SafeRelease(pBodyFrame);

		cv::imshow("Body", bodyMat);
		cv::imshow("Depth", depthMat);
		cv::imshow("Color", bufferColorMat);

		++_frame_counter;
		if (_frame_counter == 15)
		{
			double current_tick = cv::getTickCount();
			_frame_rate = _frame_counter / ((current_tick - _last_tick) / cv::getTickFrequency());
			_last_tick = current_tick;
			_frame_counter = 0;
			printf("FrameRate %.2f\n", _frame_rate);
		}

		if (cv::waitKey(10) == VK_ESCAPE){
			break;
		}
	}

	SafeRelease(pColorSource);
	SafeRelease(pBodySource);
	SafeRelease(pColorReader);
	SafeRelease(pBodyReader);
	SafeRelease(pColorDescription);
	SafeRelease(pDepthDescription);
	SafeRelease(pCoordinateMapper);
	if (pSensor){
		pSensor->Close();
	}
	SafeRelease(pSensor);
	cv::destroyAllWindows();

	return 0;
}