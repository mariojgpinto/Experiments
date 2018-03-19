#include "Sample3.h"

#include <Windows.h>
#include <NuiApi.h>
#include <opencv2/opencv.hpp>


int main_sample3(int argc, char* argv[])
{

	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

	cv::setUseOptimized( true );

	// Kinect
	INuiSensor* pSensor;
	HRESULT hResult = S_OK;
	hResult = NuiCreateSensorByIndex( 0, &pSensor );
	if( FAILED( hResult ) ){
		std::cerr << "Error : NuiCreateSensorByIndex" << std::endl;
		return -1;
	}

	hResult = pSensor->NuiInitialize( NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON );
	if( FAILED( hResult ) ){
		std::cerr << "Error : NuiInitialize" << std::endl;
		return -1;
	}

	// Color
	HANDLE hColorEvent = INVALID_HANDLE_VALUE;
	HANDLE hColorHandle = INVALID_HANDLE_VALUE;
	hColorEvent = CreateEvent( nullptr, true, false, nullptr );
	hResult = pSensor->NuiImageStreamOpen( NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, 0, 2, hColorEvent, &hColorHandle );
	if( FAILED( hResult ) ){
		std::cerr << "Error : NuiImageStreamOpen( COLOR )" << std::endl;
		return -1;
	}

	// Depth&Player
	HANDLE hDepthPlayerEvent = INVALID_HANDLE_VALUE;
	HANDLE hDepthPlayerHandle = INVALID_HANDLE_VALUE;
	hDepthPlayerEvent = CreateEvent( nullptr, true, false, nullptr );
	hResult = pSensor->NuiImageStreamOpen( NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, NUI_IMAGE_RESOLUTION_640x480, 0, 2, hDepthPlayerEvent, &hDepthPlayerHandle );
	if( FAILED( hResult ) ){
		std::cerr << "Error : NuiImageStreamOpen( DEPTH&PLAYER )" << std::endl;
		return -1;
	}

	// Skeleton
	HANDLE hSkeletonEvent = INVALID_HANDLE_VALUE;
	hSkeletonEvent = CreateEvent( nullptr, true, false, nullptr );
	hResult = pSensor->NuiSkeletonTrackingEnable( hSkeletonEvent, 0 );
	if( FAILED( hResult ) ){
		std::cerr << "Error : NuiSkeletonTrackingEnable" << std::endl;
		return -1;
	}

	HANDLE hEvents[3] = { hColorEvent, hDepthPlayerEvent, hSkeletonEvent };

	cv::Vec3b color[7];
	color[0] = cv::Vec3b(   0,   0,   0 );
	color[1] = cv::Vec3b( 255,   0,   0 );
	color[2] = cv::Vec3b(   0, 255,   0 );
	color[3] = cv::Vec3b(   0,   0, 255 );
	color[4] = cv::Vec3b( 255, 255,   0 );
	color[5] = cv::Vec3b( 255,   0, 255 );
	color[6] = cv::Vec3b(   0, 255, 255 );

	cv::namedWindow( "Color" );
	cv::namedWindow( "Depth" );
	cv::namedWindow( "Player" );
	cv::namedWindow( "Skeleton" );

	double aux1 = NUI_IMAGE_DEPTH_MAXIMUM;
	double aux2 = NUI_IMAGE_DEPTH_MINIMUM;
	double aux3 = NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE;
	double aux4 = NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE;

	double depth_alpha = 255.0f / NUI_IMAGE_DEPTH_MAXIMUM;

	while( 1 ){
		ResetEvent( hColorEvent );
		ResetEvent( hDepthPlayerEvent );
		ResetEvent( hSkeletonEvent );
		WaitForMultipleObjects( ARRAYSIZE( hEvents ), hEvents, true, INFINITE );

		// Color
		NUI_IMAGE_FRAME pColorImageFrame = { 0 };
		hResult = pSensor->NuiImageStreamGetNextFrame( hColorHandle, 0, &pColorImageFrame );
		if( FAILED( hResult ) ){
			std::cerr << "Error : NuiImageStreamGetNextFrame( COLOR )" << std::endl;
			return -1;
		}

		// Depth
		NUI_IMAGE_FRAME pDepthPlayerImageFrame = { 0 };
		hResult = pSensor->NuiImageStreamGetNextFrame( hDepthPlayerHandle, 0, &pDepthPlayerImageFrame );
		if( FAILED( hResult ) ){
			std::cerr << "Error : NuiImageStreamGetNextFrame( DEPTH&PLAYER )" << std::endl;
			return -1;
		}

		// Skeleton
		NUI_SKELETON_FRAME pSkeletonFrame = { 0 };
		hResult = pSensor->NuiSkeletonGetNextFrame( 0, &pSkeletonFrame );
		if( FAILED( hResult ) ){
			std::cout << "Error : NuiSkeletonGetNextFrame" << std::endl;
			return -1;
		}

		// Color
		INuiFrameTexture* pColorFrameTexture = pColorImageFrame.pFrameTexture;
		NUI_LOCKED_RECT sColorLockedRect;
		pColorFrameTexture->LockRect( 0, &sColorLockedRect, nullptr, 0 );

		// Depth
		INuiFrameTexture* pDepthPlayerFrameTexture = pDepthPlayerImageFrame.pFrameTexture;
		NUI_LOCKED_RECT sDepthPlayerLockedRect;
		pDepthPlayerFrameTexture->LockRect( 0, &sDepthPlayerLockedRect, nullptr, 0 );

		cv::Mat colorMat( 480, 640, CV_8UC4, reinterpret_cast<uchar*>( sColorLockedRect.pBits ) );
		cv::Mat depthMat( 480, 640, CV_16SC1, reinterpret_cast<uchar*>( sDepthPlayerLockedRect.pBits )  );

		//LONG registX = 0;
		//LONG registY = 0;
		ushort* pBuffer = reinterpret_cast<ushort*>( sDepthPlayerLockedRect.pBits );
		//cv::Mat bufferMat = cv::Mat::zeros( 480, 640, CV_16UC1 );
		//cv::Mat playerMat = cv::Mat::zeros( 480, 640, CV_8UC3 );
		//for( int y = 0; y < 480; y++ ){
		//	for( int x = 0; x < 640; x++ ){
		//		pSensor->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution( NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, nullptr, x, y, *pBuffer, &registX, &registY );
		//		if( ( registX >= 0 ) && ( registX < 640 ) && ( registY >= 0 ) && ( registY < 480 ) ){
		//			bufferMat.at<ushort>( registY, registX ) = *pBuffer & 0xFFF8;
		//			playerMat.at<cv::Vec3b>( registY, registX ) = color[*pBuffer & 0x7];
		//		}
		//		pBuffer++;
		//	}
		//}
		//cv::Mat depthMat( 480, 640, CV_8UC1 );
		//bufferMat.convertTo( depthMat, CV_8UC3, -255.0f / NUI_IMAGE_DEPTH_MAXIMUM, 255.0f );

		cv::Mat skeletonMat = cv::Mat::zeros( 480, 640, CV_8UC3 );
		cv::Point2f point;
		cv::Point3f point_hand;
		for( int count = 0; count < NUI_SKELETON_COUNT; count++ ){
			NUI_SKELETON_DATA skeleton = pSkeletonFrame.SkeletonData[count];
			if( skeleton.eTrackingState == NUI_SKELETON_TRACKED ){
				for( int position = 0; position < NUI_SKELETON_POSITION_COUNT; position++ ){
					NuiTransformSkeletonToDepthImage( skeleton.SkeletonPositions[position], &point.x, &point.y, NUI_IMAGE_RESOLUTION_640x480 );
					cv::circle( skeletonMat, point, 10, static_cast<cv::Scalar>( color[count + 1] ), -1, CV_AA );

					if(position == NUI_SKELETON_POSITION_HAND_RIGHT){
						point_hand.x = skeleton.SkeletonPositions[position].x;
						point_hand.y = skeleton.SkeletonPositions[position].y;
						point_hand.z = skeleton.SkeletonPositions[position].z;
					}
				}
			}
		}	


		cv::Point p1(320,240);
		cv::Point p2(500,240);

		Vector4 vec =  NuiTransformDepthImageToSkeleton(p1.x/640,p1.y/480,NuiDepthPixelToDepth(pBuffer[p1.y*640+p1.x]) << 3);

		char buff1[100];
		sprintf(buff1,"X:%.2f  Y:%.2f  Z:%.2f  A:%.2f", vec.x, vec.y, vec.z, vec.w);
		cv::putText(skeletonMat, buff1, cvPoint(30,30), cv::FONT_HERSHEY_COMPLEX, 1., cvScalar(255,255,255), 1, CV_AA);


		char buff2[100];
		sprintf(buff2,"Distance(mm):%d , %d", NuiDepthPixelToDepth(pBuffer[p1.y*640+p1.x]),pBuffer[p1.y*640+p1.x]);
		cv::putText(skeletonMat, buff2, cvPoint(30,100), cv::FONT_HERSHEY_COMPLEX, 1., cvScalar(255,255,255), 1, CV_AA);

		char buff3[100];
		sprintf(buff3,"Distance(mm):%d , %d", NuiDepthPixelToDepth(pBuffer[p2.y*640+p2.x]),pBuffer[p2.y*640+p2.x]);
		cv::putText(skeletonMat, buff3, cvPoint(30,200), cv::FONT_HERSHEY_COMPLEX, 1., cvScalar(255,255,255), 1, CV_AA);

		

		cv::Mat temp;
		colorMat.convertTo(temp,CV_8UC3);
		// //= colorMat + (0.5*skeletonMat);
		cv::imshow( "Color", temp);

		cv::Mat depthMat8UC1;
		depthMat.convertTo( depthMat8UC1, CV_8UC1, depth_alpha);



		cv::circle(depthMat8UC1,p1,5,cv::Scalar(255,255,255),-1);
		cv::circle(depthMat8UC1,p2,5,cv::Scalar(255,255,255),-1);

		cv::imshow( "Depth", depthMat8UC1 );
		
		cv::Mat temp2;
		temp.copyTo(temp2,depthMat8UC1);

		cv::imshow( "Player", temp2 );
		
		
		cv::imshow( "Skeleton", skeletonMat );

		pColorFrameTexture->UnlockRect( 0 );
		pDepthPlayerFrameTexture->UnlockRect( 0 );
		pSensor->NuiImageStreamReleaseFrame( hColorHandle, &pColorImageFrame );
		pSensor->NuiImageStreamReleaseFrame( hDepthPlayerHandle, &pDepthPlayerImageFrame );

		if( cv::waitKey( 11 ) == VK_ESCAPE ){
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

	pSensor->NuiShutdown();
	pSensor->NuiSkeletonTrackingDisable();
	CloseHandle( hColorEvent );
	CloseHandle( hDepthPlayerEvent );
	CloseHandle( hSkeletonEvent );
	CloseHandle( hColorHandle );
	CloseHandle( hDepthPlayerHandle );

	cv::destroyAllWindows();

	return 0;
}