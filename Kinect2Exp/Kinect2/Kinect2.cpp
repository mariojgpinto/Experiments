#include "Kinect2.h"

#include <iostream>
#include <sstream>

#include <Windows.h>
#include <Kinect.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
    if (pInterfaceToRelease != NULL)
    {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}

int main_kinect_2(int argc, char** argv) {
	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;
		
	   HRESULT hr;

	IKinectSensor*          m_pKinectSensor;
	IMultiSourceFrameReader*m_pMultiSourceFrameReader;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pKinectSensor)
    {
        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_BodyIndex,
                &m_pMultiSourceFrameReader);
        }
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        //SetStatusMessage(L"No ready Kinect found!", 10000, true);
        return E_FAIL;
    }
	

    IMultiSourceFrame* pMultiSourceFrame = NULL;
    IDepthFrame* pDepthFrame = NULL;
	IFrameDescription *frameDesc = nullptr;
	int height = 424, width = 512;
	USHORT nDepthMinReliableDistance = 0;
    USHORT nDepthMaxReliableDistance = 0;
    UINT16 *depthBuffer = nullptr;
	
	IColorFrame* pColorFrame = NULL;
	
	   

	while(true){
		HRESULT hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);

		if (SUCCEEDED(hr)){
			IDepthFrameReference* pDepthFrameReference = NULL;

			hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
			if (SUCCEEDED(hr)){
				hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
			}

			SafeRelease(pDepthFrameReference);
		}

		if (SUCCEEDED(hr))
		{
			IColorFrameReference* pColorFrameReference = NULL;

			hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
			if (SUCCEEDED(hr))
			{
				hr = pColorFrameReference->AcquireFrame(&pColorFrame);
			}

			SafeRelease(pColorFrameReference);
		}

		if (SUCCEEDED(hr))
		{
			INT64 nDepthTime = 0;
			IFrameDescription* pDepthFrameDescription = NULL;
			int nDepthWidth = 0;
			int nDepthHeight = 0;
			UINT nDepthBufferSize = 0;
			UINT16 *pDepthBuffer = NULL;

			IFrameDescription* pColorFrameDescription = NULL;
			int nColorWidth = 0;
			int nColorHeight = 0;
			ColorImageFormat imageFormat = ColorImageFormat_None;
			UINT nColorBufferSize = 0;
			RGBQUAD *pColorBuffer = NULL;

			// get depth frame data

			hr = pDepthFrame->get_RelativeTime(&nDepthTime);

			if (SUCCEEDED(hr)){
				hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);
			}

			if (SUCCEEDED(hr)){
				hr = pDepthFrameDescription->get_Width(&nDepthWidth);
			}

			if (SUCCEEDED(hr)){
				hr = pDepthFrameDescription->get_Height(&nDepthHeight);
			}

			if (SUCCEEDED(hr)){
				hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);            
			}

			// get color frame data

			if (SUCCEEDED(hr))
			{
				hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);
			}

			if (SUCCEEDED(hr))
			{
				hr = pColorFrameDescription->get_Width(&nColorWidth);
			}

			if (SUCCEEDED(hr))
			{
				hr = pColorFrameDescription->get_Height(&nColorHeight);
			}

			if (SUCCEEDED(hr))
			{
				hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
			}

			if (SUCCEEDED(hr))
			{
				if (imageFormat == ColorImageFormat_Bgra)
				{
					hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
				}
				//else if (m_pColorRGBX)
				//{
				//	pColorBuffer = m_pColorRGBX;
				//	nColorBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
				//	hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
				//}
				//else
				//{
				//	hr = E_FAIL;
				//}
			}

			SafeRelease(pDepthFrameDescription);
			 SafeRelease(pColorFrameDescription);
		}


		SafeRelease(pDepthFrame);
		SafeRelease(pColorFrame);
		//SafeRelease(pBodyIndexFrame);
		SafeRelease(pMultiSourceFrame);

		++_frame_counter;
		if (_frame_counter == 15)
		{
			double current_tick = cv::getTickCount();
			_frame_rate = _frame_counter / ((current_tick - _last_tick)/cv::getTickFrequency());
			_last_tick = current_tick;
			_frame_counter = 0;
			printf("FrameRate %.2f\n",_frame_rate);
		}

		if (cv::waitKey(11) == 27){
			break;
		}
    }


    return 0;
}


/*
inline void CHECKERROR(HRESULT n) {
    if (!SUCCEEDED(n)) {
        std::stringstream ss;
        ss << "ERROR " << std::hex << n << std::endl;
        std::cin.ignore();
        std::cin.get();
        throw std::runtime_error(ss.str().c_str());
    }
}

// Safe release for interfaces
//inline void SAFERELEASE(IInterface *& pInterfaceToRelease) {
//    if (pInterfaceToRelease != nullptr) {
//        pInterfaceToRelease->Release();
//        pInterfaceToRelease = nullptr;
//    }
//}

IDepthFrameReader* depthFrameReader = nullptr; // depth reader

void processIncomingDepthData() {
    IDepthFrame *data = nullptr;
    IFrameDescription *frameDesc = nullptr;
    HRESULT hr = -1;
    UINT16 *depthBuffer = nullptr;
    USHORT nDepthMinReliableDistance = 0;
    USHORT nDepthMaxReliableDistance = 0;
    int height = 424, width = 512;

    hr = depthFrameReader->AcquireLatestFrame(&data);
    if (SUCCEEDED(hr)) hr = data->get_FrameDescription(&frameDesc);
    if (SUCCEEDED(hr)) hr = data->get_DepthMinReliableDistance(
        &nDepthMinReliableDistance);
    if (SUCCEEDED(hr)) hr = data->get_DepthMaxReliableDistance(
        &nDepthMaxReliableDistance);

    if (SUCCEEDED(hr)) {
            if (SUCCEEDED(frameDesc->get_Height(&height)) &&
            SUCCEEDED(frameDesc->get_Width(&width))) {
            depthBuffer = new UINT16[height * width];
            hr = data->CopyFrameDataToArray(height * width, depthBuffer);
            if (SUCCEEDED(hr)) {
                cv::Mat depthMap = cv::Mat(height, width, CV_16U, depthBuffer);
                cv::Mat img0 = cv::Mat::zeros(height, width, CV_8UC1);
                cv::Mat img1;
                double scale = 255.0 / (nDepthMaxReliableDistance - 
                    nDepthMinReliableDistance);
                depthMap.convertTo(img0, CV_8UC1, scale);
                //applyColorMap(img0, img1, cv::COLORMAP_HOT);
                cv::imshow("Depth Only", img0);
            }
        }
    }
    if (depthBuffer != nullptr) {
        delete[] depthBuffer;
        depthBuffer = nullptr;
    }
   if (data != nullptr) {
        data->Release();
        data = nullptr;
    }
}


IColorFrameReader* colorFrameReader = nullptr; // depth reader

void processIncomingColorData() {
    IColorFrame *data = nullptr;
    IFrameDescription *frameDesc= nullptr;
    HRESULT hr = -1;
    BYTE *colorBuffer = nullptr;
    int height= 1080, width= 1920;

    hr = colorFrameReader->AcquireLatestFrame(&data);
    if (SUCCEEDED(hr)) hr = data->get_FrameDescription(&frameDesc);

    if (SUCCEEDED(hr)) {
            if (SUCCEEDED(frameDesc->get_Height(&height)) &&
            SUCCEEDED(frameDesc->get_Width(&width))) {
            colorBuffer = new BYTE[height * width];
			hr = data->CopyConvertedFrameDataToArray(height * width, colorBuffer, ColorImageFormat_Yuv);
            if (SUCCEEDED(hr)) {
				cv::Mat colorMap = cv::Mat(height, width, CV_8UC3, colorBuffer);
                cv::Mat img1;

                cv::imshow("Color Only", colorMap);
            }
        }
    }
    if (colorBuffer != nullptr) {
        delete[] colorBuffer;
        colorBuffer = nullptr;
    }
   if (data != nullptr) {
        data->Release();
        data = nullptr;
    }
}

int main_kinect_2(int argc, char** argv) {
	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

    HRESULT hr;
    IKinectSensor* kinectSensor = nullptr;     // kinect sensor

    // initialize Kinect Sensor
    hr = GetDefaultKinectSensor(&kinectSensor);
    if (FAILED(hr) || !kinectSensor) {
        std::cout << "ERROR hr=" << hr << "; sensor=" << kinectSensor << std::endl;
        return -1;
    }
    CHECKERROR(kinectSensor->Open());
	
    // initialize depth frame reader
    IDepthFrameSource* depthFrameSource = nullptr;
    CHECKERROR(kinectSensor->get_DepthFrameSource(&depthFrameSource));
    CHECKERROR(depthFrameSource->OpenReader(&depthFrameReader));
      if (depthFrameSource != nullptr) {
        depthFrameSource->Release();
        depthFrameSource = nullptr;
    }

	   // initialize color frame reader
    IColorFrameSource* colorFrameSource = nullptr;
    CHECKERROR(kinectSensor->get_ColorFrameSource(&colorFrameSource));
    CHECKERROR(colorFrameSource->OpenReader(&colorFrameReader));
      if (colorFrameSource != nullptr) {
        colorFrameSource->Release();
        colorFrameSource = nullptr;
    }


    while (depthFrameReader && colorFrameReader) {
        processIncomingDepthData();
		processIncomingColorData();
        int key = cv::waitKey(10);
        if (key == 'q'){
            break;
        }

		++_frame_counter;
		if (_frame_counter == 15)
		{
			double current_tick = cv::getTickCount();
			_frame_rate = _frame_counter / ((current_tick - _last_tick)/cv::getTickFrequency());
			_last_tick = current_tick;
			_frame_counter = 0;
			printf("FrameRate %.2f\n",_frame_rate);
		}
    }

    // de-initialize Kinect Sensor
    CHECKERROR(kinectSensor->Close());
     if (kinectSensor != nullptr) {
        kinectSensor->Release();
        kinectSensor = nullptr;
    }
    return 0;
}*/