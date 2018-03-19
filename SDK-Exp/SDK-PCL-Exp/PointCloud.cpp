#include "PointCloud.h"

#include <Windows.h>
#include <NuiApi.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

int main_pcl_point_cloud_v3(int argc, char* argv[]){
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

	hResult = pSensor->NuiInitialize( NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH);
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

	// Depth
	HANDLE hDepthEvent = INVALID_HANDLE_VALUE;
	HANDLE hDepthHandle = INVALID_HANDLE_VALUE;
	hDepthEvent = CreateEvent( nullptr, true, false, nullptr );
	hResult = pSensor->NuiImageStreamOpen( NUI_IMAGE_TYPE_DEPTH , NUI_IMAGE_RESOLUTION_640x480, 0, 2, hDepthEvent, &hDepthHandle );
	if( FAILED( hResult ) ){
		std::cerr << "Error : NuiImageStreamOpen( DEPTH )" << std::endl;
		return -1;
	}

	HANDLE hEvents[2] = { hColorEvent, hDepthEvent };

	cv::namedWindow( "Color" );
	cv::namedWindow( "Depth" );
	cv::namedWindow( "Mix" );

	double depth_alpha = 255.0f / NUI_IMAGE_DEPTH_MAXIMUM;

	pcl::visualization::CloudViewer viewer("Simple cloud_file Viewer");
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	cloud.width = 640*480;
	cloud.height = 1;
	cloud.points.resize (cloud.width * cloud.height);
				
	INuiCoordinateMapper *mapper;
	pSensor->NuiGetCoordinateMapper(&mapper);

	NUI_COLOR_IMAGE_POINT buff[640*480];
	Vector4 buff_skeleton[640*480];

	while( 1 ){
		ResetEvent( hColorEvent );
		ResetEvent( hDepthEvent );
		WaitForMultipleObjects( ARRAYSIZE( hEvents ), hEvents, true, INFINITE );

		// Color
		NUI_IMAGE_FRAME pColorImageFrame = { 0 };
		hResult = pSensor->NuiImageStreamGetNextFrame( hColorHandle, 0, &pColorImageFrame );
		if( FAILED( hResult ) ){
			std::cerr << "Error : NuiImageStreamGetNextFrame( COLOR )" << std::endl;
			return -1;
		}

		// Depth
		NUI_IMAGE_FRAME pDepthImageFrame = { 0 };
		hResult = pSensor->NuiImageStreamGetNextFrame( hDepthHandle, 0, &pDepthImageFrame );
		if( FAILED( hResult ) ){
			std::cerr << "Error : NuiImageStreamGetNextFrame( DEPTH&PLAYER )" << std::endl;
			return -1;
		}

		
		// Color
		INuiFrameTexture* pColorFrameTexture = pColorImageFrame.pFrameTexture;
		NUI_LOCKED_RECT sColorLockedRect;
		pColorFrameTexture->LockRect( 0, &sColorLockedRect, nullptr, 0 );

		 // Depth
		INuiFrameTexture* pDepthFrameTexture = pDepthImageFrame.pFrameTexture;
		NUI_LOCKED_RECT sDepthLockedRect;
		pDepthFrameTexture->LockRect( 0, &sDepthLockedRect, nullptr, 0 );

		mapper->MapDepthFrameToColorFrame(NUI_IMAGE_RESOLUTION_640x480, sDepthLockedRect.size, reinterpret_cast<NUI_DEPTH_IMAGE_PIXEL*>(sDepthLockedRect.pBits), NUI_IMAGE_TYPE_COLOR ,NUI_IMAGE_RESOLUTION_640x480,640*480, buff);

		//mapper->MapDepthPointToColorPoint(NUI_IMAGE_RESOLUTION_640x480, reinterpret_cast<NUI_DEPTH_IMAGE_POINT*>(sDepthLockedRect.pBits), NUI_IMAGE_TYPE_COLOR ,NUI_IMAGE_RESOLUTION_640x480, buff);


		//mapper->MapColorFrameToSkeletonFrame(NUI_IMAGE_TYPE_COLOR,NUI_IMAGE_RESOLUTION_640x480,NUI_IMAGE_RESOLUTION_640x480,640*480,

		cv::Mat colorMat( 480, 640, CV_8UC4 , reinterpret_cast<uchar*>( sColorLockedRect.pBits ) );
		cv::Mat depthMat( 480, 640, CV_16SC1, reinterpret_cast<uchar*>( sDepthLockedRect.pBits ) );
			 
		ushort* pBuffer = reinterpret_cast<ushort*>( sDepthLockedRect.pBits );
	
		cv::Mat color;
		colorMat.convertTo(color,CV_8UC3);

		cloud.points.clear();
		//viewer.removeAllPointClouds();

		LONG registX = 0;
		LONG registY = 0;
		uchar* ptr_clr = (uchar*)color.data;

		int idx = 0;
		for(int y = 0 ; y < 480 ; y+=1) { 
			for(int x = 0 ; x < 640 ; x+=1) {
				idx = y*640+x;
				if(pBuffer[idx] > 0){
					Vector4 vec =  NuiTransformDepthImageToSkeleton(x,y,pBuffer[idx],NUI_IMAGE_RESOLUTION_640x480);
				
					//pSensor->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution( NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, nullptr, x, y, pBuffer[y*640+x], &registX, &registY );
					
					//pcl::PointXYZRGB pt(255,255,255);
					//if(buff[idx].x > 0 && buff[idx].y > 0){
					//	int cpos = buff[idx].y *640 * 4 + buff[idx].x*4;
					//
					//	pt.r = ptr_clr[cpos+2];
					//	pt.g = ptr_clr[cpos+1];
					//	pt.b = ptr_clr[cpos  ];
					//}

					//pcl::PointXYZRGB pt(255,255,255);

					pSensor->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution( NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, nullptr, x, y, pBuffer[idx], &registX, &registY );
					int cpos = registY * 640 * 4 + registX* 4;					
					pcl::PointXYZRGB pt(sColorLockedRect.pBits[cpos+2],
										sColorLockedRect.pBits[cpos+1],
										sColorLockedRect.pBits[cpos  ]);
					pt.x = vec.x;
					pt.y = -vec.y;
					pt.z = vec.z;

					cloud.push_back(pt);
					
					//cloud.push_back(pcl::PointXYZ(x,y,vec.z));
				}
			}
		}

		

		//viewer.addPointCloud(cloud.makeShared());
		//viewer.spinOnce (10);

		viewer.showCloud(cloud.makeShared());

		cv::imshow( "Color", color);

		cv::Mat depthMat8UC1;
		depthMat.convertTo( depthMat8UC1, CV_8UC1, depth_alpha);
		cv::imshow( "Depth", depthMat8UC1 );
		
		cv::Mat color_masked;
		color.copyTo(color_masked,depthMat8UC1);
		cv::imshow( "Mix", color_masked);
						   
		pColorFrameTexture->UnlockRect( 0 );
		pDepthFrameTexture->UnlockRect( 0 );
		pSensor->NuiImageStreamReleaseFrame( hColorHandle, &pColorImageFrame );
		pSensor->NuiImageStreamReleaseFrame( hDepthHandle, &pDepthImageFrame );


		if( cv::waitKey(1) == VK_ESCAPE ){
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
	CloseHandle( hColorEvent );
	CloseHandle( hDepthEvent );
	//CloseHandle( hColorHandle );
	//CloseHandle( hDepthHandle );

	cv::destroyAllWindows();

	return 0;
}


int main_pcl_point_cloud_v2(int argc, char* argv[]){
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

	hResult = pSensor->NuiInitialize( NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH);
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

	// Depth
	HANDLE hDepthEvent = INVALID_HANDLE_VALUE;
	HANDLE hDepthHandle = INVALID_HANDLE_VALUE;
	hDepthEvent = CreateEvent( nullptr, true, false, nullptr );
	hResult = pSensor->NuiImageStreamOpen( NUI_IMAGE_TYPE_DEPTH , NUI_IMAGE_RESOLUTION_640x480, 0, 2, hDepthEvent, &hDepthHandle );
	if( FAILED( hResult ) ){
		std::cerr << "Error : NuiImageStreamOpen( DEPTH )" << std::endl;
		return -1;
	}

	HANDLE hEvents[2] = { hColorEvent, hDepthEvent };

	cv::namedWindow( "Color" );
	cv::namedWindow( "Depth" );
	cv::namedWindow( "Mix" );

	double depth_alpha = 255.0f / NUI_IMAGE_DEPTH_MAXIMUM;

	pcl::visualization::CloudViewer viewer("Simple cloud_file Viewer");
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	cloud.width = 640*480;
	cloud.height = 1;
	cloud.points.resize (cloud.width * cloud.height);

	while( 1 ){
		ResetEvent( hColorEvent );
		ResetEvent( hDepthEvent );
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
		hResult = pSensor->NuiImageStreamGetNextFrame( hDepthHandle, 0, &pDepthPlayerImageFrame );
		if( FAILED( hResult ) ){
			std::cerr << "Error : NuiImageStreamGetNextFrame( DEPTH&PLAYER )" << std::endl;
			return -1;
		}

		
		// Color
		INuiFrameTexture* pColorFrameTexture = pColorImageFrame.pFrameTexture;
		NUI_LOCKED_RECT sColorLockedRect;
		pColorFrameTexture->LockRect( 0, &sColorLockedRect, nullptr, 0 );

		// Depth
		INuiFrameTexture* pDepthFrameTexture = pDepthPlayerImageFrame.pFrameTexture;
		NUI_LOCKED_RECT sDepthPlayerLockedRect;
		pDepthFrameTexture->LockRect( 0, &sDepthPlayerLockedRect, nullptr, 0 );

		cv::Mat colorMat( 480, 640, CV_8UC4, reinterpret_cast<uchar*>( sColorLockedRect.pBits ) );
		cv::Mat depthMat( 480, 640, CV_16SC1, reinterpret_cast<uchar*>( sDepthPlayerLockedRect.pBits )  );


		ushort* pBuffer = reinterpret_cast<ushort*>( sDepthPlayerLockedRect.pBits );
	
		cv::Mat color;
		colorMat.convertTo(color,CV_8UC3);

		cloud.points.clear();

		LONG registX = 0;
		LONG registY = 0;
		uchar* ptr_clr = (uchar*)color.data;
		for(int y = 0 ; y < 480 ; y+=1) { 
			for(int x = 0 ; x < 640 ; x+=1) {
				if(pBuffer[y*640+x] > 0){
					//short depth = (short)(pBuffer[y*640+x] >> 3 | pBuffer[y*640+x + 1] << 5);
					
					//Vector4 vec =  NuiTransformDepthImageToSkeleton(p1.x/640.0f,p1.y/480.0f,NuiDepthPixelToDepth(pBuffer[y*640+x]) << 3);
					Vector4 vec =  NuiTransformDepthImageToSkeleton(x,y,pBuffer[y*640+x],NUI_IMAGE_RESOLUTION_640x480);
				
					//pSensor->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution( NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, nullptr, x, y, pBuffer[y*640+x], &registX, &registY );
					//int cpos = registY*640 * 4 + registX*4;

					//pcl::PointXYZRGB pt(ptr_clr[cpos+2],
					//					ptr_clr[cpos+1],
					//					ptr_clr[cpos  ]);

					pcl::PointXYZRGB pt(255,255,255);

					//pSensor->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution( NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, nullptr, x, y, pBuffer[y*640+x], &registX, &registY );
					//int cpos = registY * 640 * 4 + registX* 4;					
					//pcl::PointXYZRGB pt(sColorLockedRect.pBits[cpos+2],
					//					sColorLockedRect.pBits[cpos+1],
					//					sColorLockedRect.pBits[cpos  ]);
					pt.x = vec.x;
					pt.y = -vec.y;
					pt.z = vec.z;

					cloud.push_back(pt);
					
					//cloud.push_back(pcl::PointXYZ(x,y,vec.z));
				}
			}
		}

		viewer.showCloud(cloud.makeShared());

		cv::imshow( "Color", color);

		cv::Mat depthMat8UC1;
		depthMat.convertTo( depthMat8UC1, CV_8UC1, depth_alpha);
		cv::imshow( "Depth", depthMat8UC1 );
		
		cv::Mat color_masked;
		color.copyTo(color_masked,depthMat8UC1);
		cv::imshow( "Mix", color_masked);
						   
		pColorFrameTexture->UnlockRect( 0 );
		pDepthFrameTexture->UnlockRect( 0 );
		pSensor->NuiImageStreamReleaseFrame( hColorHandle, &pColorImageFrame );
		pSensor->NuiImageStreamReleaseFrame( hDepthHandle, &pDepthPlayerImageFrame );


		if( cv::waitKey(1) == VK_ESCAPE ){
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
	CloseHandle( hColorEvent );
	CloseHandle( hDepthEvent );
	//CloseHandle( hColorHandle );
	//CloseHandle( hDepthHandle );

	cv::destroyAllWindows();

	return 0;
}

int main_pcl_point_cloud_v1(int argc, char* argv[])
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

	pcl::visualization::CloudViewer viewer("Simple cloud_file Viewer");
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	cloud.width = 640*480;
	cloud.height = 1;
	cloud.points.resize (cloud.width * cloud.height);

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

		//cv::Mat skeletonMat = cv::Mat::zeros( 480, 640, CV_8UC3 );
		//cv::Point2f point;
		//cv::Point3f point_hand;
		//for( int count = 0; count < NUI_SKELETON_COUNT; count++ ){
		//	NUI_SKELETON_DATA skeleton = pSkeletonFrame.SkeletonData[count];
		//	if( skeleton.eTrackingState == NUI_SKELETON_TRACKED ){
		//		for( int position = 0; position < NUI_SKELETON_POSITION_COUNT; position++ ){
		//			NuiTransformSkeletonToDepthImage( skeleton.SkeletonPositions[position], &point.x, &point.y, NUI_IMAGE_RESOLUTION_640x480 );
		//			cv::circle( skeletonMat, point, 10, static_cast<cv::Scalar>( color[count + 1] ), -1, CV_AA );

		//			if(position == NUI_SKELETON_POSITION_HAND_RIGHT){
		//				point_hand.x = skeleton.SkeletonPositions[position].x;
		//				point_hand.y = skeleton.SkeletonPositions[position].y;
		//				point_hand.z = skeleton.SkeletonPositions[position].z;
		//			}
		//		}
		//	}
		//}	

		cv::Mat color;
		colorMat.convertTo(color,CV_8UC3);

		cloud.points.clear();

		LONG registX = 0;
		LONG registY = 0;
		uchar* ptr_clr = (uchar*)color.data;
		for(int y = 0 ; y < 480 ; y+=1) { 
			for(int x = 0 ; x < 640 ; x+=1) { 

				if(pBuffer[y*640+x] > 0){
					//short depth = (short)(pBuffer[y*640+x] >> 3 | pBuffer[y*640+x + 1] << 5);
					
					//Vector4 vec =  NuiTransformDepthImageToSkeleton(p1.x/640.0f,p1.y/480.0f,NuiDepthPixelToDepth(pBuffer[y*640+x]) << 3);
					Vector4 vec =  NuiTransformDepthImageToSkeleton(x,y,pBuffer[y*640+x],NUI_IMAGE_RESOLUTION_640x480);
				
					pSensor->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution( NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, nullptr, x, y, pBuffer[y*640+x], &registX, &registY );
					int cpos = registY*640 * 4 + registX*4;

					pcl::PointXYZRGB pt(ptr_clr[cpos+2],
										ptr_clr[cpos+1],
										ptr_clr[cpos  ]);

					//pSensor->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution( NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, nullptr, x, y, pBuffer[y*640+x], &registX, &registY );
					//int cpos = registY * 640 * 4 + registX* 4;					
					//pcl::PointXYZRGB pt(sColorLockedRect.pBits[cpos+2],
					//					sColorLockedRect.pBits[cpos+1],
					//					sColorLockedRect.pBits[cpos  ]);
					pt.x = vec.x;
					pt.y = -vec.y;
					pt.z = vec.z;

					cloud.push_back(pt);
					
					//cloud.push_back(pcl::PointXYZ(x,y,vec.z));
				}
			}
		}

		viewer.showCloud(cloud.makeShared());

		//cv::Point p1(320,240);
		//cv::Point p2(500,240);

		//Vector4 vec =  NuiTransformDepthImageToSkeleton(p1.x/640,p1.y/480,NuiDepthPixelToDepth(pBuffer[p1.y*640+p1.x]) << 3);

		//char buff1[100];
		//sprintf(buff1,"X:%.2f  Y:%.2f  Z:%.2f  A:%.2f", vec.x, vec.y, vec.z, vec.w);
		//cv::putText(skeletonMat, buff1, cvPoint(30,30), cv::FONT_HERSHEY_COMPLEX, 1., cvScalar(255,255,255), 1, CV_AA);


		//char buff2[100];
		//sprintf(buff2,"Distance(mm):%d , %d", NuiDepthPixelToDepth(pBuffer[p1.y*640+p1.x]),pBuffer[p1.y*640+p1.x]);
		//cv::putText(skeletonMat, buff2, cvPoint(30,100), cv::FONT_HERSHEY_COMPLEX, 1., cvScalar(255,255,255), 1, CV_AA);

		//char buff3[100];
		//sprintf(buff3,"Distance(mm):%d , %d", NuiDepthPixelToDepth(pBuffer[p2.y*640+p2.x]),pBuffer[p2.y*640+p2.x]);
		//cv::putText(skeletonMat, buff3, cvPoint(30,200), cv::FONT_HERSHEY_COMPLEX, 1., cvScalar(255,255,255), 1, CV_AA);

		//char buff4[100];
		//sprintf(buff4,"X[%.2f .. %.2f]", min_x, max_x);
		//cv::putText(skeletonMat, buff4, cvPoint(30,275), cv::FONT_HERSHEY_COMPLEX, 1., cvScalar(255,255,255), 1, CV_AA);
		//char buff5[100];
		//sprintf(buff5,"Y[%.2f .. %.2f]", min_y, max_y);
		//cv::putText(skeletonMat, buff5, cvPoint(30,325), cv::FONT_HERSHEY_COMPLEX, 1., cvScalar(255,255,255), 1, CV_AA);
		//char buff6[100];
		//sprintf(buff6,"Z[%.2f .. %.2f]", min_z, max_z);
		//cv::putText(skeletonMat, buff6, cvPoint(30,375), cv::FONT_HERSHEY_COMPLEX, 1., cvScalar(255,255,255), 1, CV_AA);

		

		//cv::Mat depthMat8UC1;
		//depthMat.convertTo( depthMat8UC1, CV_8UC1, depth_alpha);
		//
		//cv::imshow( "Depth", depthMat8UC1 );
		//
		//cv::Mat temp2;
		//color.copyTo(temp2,depthMat8UC1);

		//cv::imshow( "Player", temp2 );
		//
		//
		//cv::imshow( "Skeleton", skeletonMat );

		pColorFrameTexture->UnlockRect( 0 );
		pDepthPlayerFrameTexture->UnlockRect( 0 );
		pSensor->NuiImageStreamReleaseFrame( hColorHandle, &pColorImageFrame );
		pSensor->NuiImageStreamReleaseFrame( hDepthPlayerHandle, &pDepthPlayerImageFrame );

		cv::imshow( "Color", color);

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
	//CloseHandle( hSkeletonEvent );
	//CloseHandle( hColorHandle );
	//CloseHandle( hDepthPlayerHandle );

	cv::destroyAllWindows();

	return 0;
}