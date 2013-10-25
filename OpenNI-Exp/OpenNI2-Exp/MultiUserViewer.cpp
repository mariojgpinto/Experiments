#include "MultiUserViewer.h"

#include <OpenNI.h>
#include <NiTE.h>
#include <opencv2\opencv.hpp>

const bool flag_color = true;

int main_multi_user_viewer(int argc, char* argv[]){
	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

	int color_step = (flag_color) ? 2 : 1;

	openni::Status rc = openni::STATUS_OK;

	rc = openni::OpenNI::initialize();
	if (rc != openni::STATUS_OK){
		printf("%s: Initialize failed\n%s\n", argv[0], openni::OpenNI::getExtendedError());
		return 1;
	}

	openni::Array<openni::DeviceInfo> deviceList;
	openni::OpenNI::enumerateDevices(&deviceList);

	int n_kinects = deviceList.getSize();

	//****************************************************************
	//* CREATE DEVICE VARIABLES
	//****************************************************************
	openni::Device** devices = (openni::Device**)malloc(sizeof(openni::Device*) * n_kinects);
	for(int i = 0 ; i < n_kinects ; ++i)
		devices[i] = new openni::Device();

	openni::VideoStream** depth = (openni::VideoStream**)malloc(sizeof(openni::VideoStream*) * n_kinects);
	for(int i = 0 ; i < n_kinects ; ++i)
		depth[i] = new openni::VideoStream();

	openni::VideoStream** color = (openni::VideoStream**)malloc(sizeof(openni::VideoStream*) * n_kinects);
	for(int i = 0 ; i < n_kinects ; ++i)
		color[i] = new openni::VideoStream();

	//****************************************************************
	//* OPEN KINECTS
	//****************************************************************
	for(int i = 0 ; i < n_kinects ; ++i){
		//************************ INIT ******************************
		rc = devices[i]->open(deviceList[i].getUri());
		if (rc != openni::STATUS_OK){
			printf("%s: Couldn't open device %s\n%s\n", argv[0], deviceList[i], openni::OpenNI::getExtendedError());
			openni::OpenNI::shutdown();
			return 3;
		}


		//*********************** DEPTH *****************************
		rc = depth[i]->create(*devices[i], openni::SENSOR_DEPTH);
		if (rc == openni::STATUS_OK){
			rc = depth[i]->start();

			if (rc == openni::STATUS_OK){
				if (!depth[i]->isValid()){
					printf("SimpleViewer: No valid streams. Exiting\n");
					openni::OpenNI::shutdown();
					return 6;
				}
			}
			else{
				printf("%s: Couldn't start stream %d on device %s\n%s\n", argv[0], openni::SENSOR_DEPTH, deviceList[i], openni::OpenNI::getExtendedError());
				openni::OpenNI::shutdown();
				return 5;
			}
		}
		else{
			printf("%s: Couldn't create stream %d on device %s\n%s\n", argv[0], openni::SENSOR_DEPTH, deviceList[i], openni::OpenNI::getExtendedError());
			openni::OpenNI::shutdown();
			return 4;
		}		
		
		//*********************** COLOR *****************************
		if(flag_color){
			rc = color[i]->create(*devices[i], openni::SENSOR_COLOR);
			if (rc == openni::STATUS_OK){
				rc = color[i]->start();

				if (rc == openni::STATUS_OK){
					if (!color[i]->isValid()){
						printf("SimpleViewer: No valid streams. Exiting\n");
						openni::OpenNI::shutdown();
						return 6;
					}
				}
				else{
					printf("%s: Couldn't start stream %d on device %s\n%s\n", argv[0], openni::SENSOR_COLOR, deviceList[i], openni::OpenNI::getExtendedError());
					openni::OpenNI::shutdown();
					return 5;
				}			
			}
			else{
				printf("%s: Couldn't create stream %d on device %s\n%s\n", argv[0], openni::SENSOR_COLOR, deviceList[i], openni::OpenNI::getExtendedError());
				openni::OpenNI::shutdown();
				return 4;
			}		
		}
	}
	
	for(int i = 0 ; i < n_kinects ; ++i){
		rc = devices[i]->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );
		//rc = devices[i]->setDepthColorSyncEnabled(true);
		//rc = devices[i]->
	}

	//****************************************************************
	//* CREATE READ VARIABLES
	//****************************************************************
	openni::VideoStream** depth_stream = (openni::VideoStream**)malloc(sizeof(openni::VideoStream*) * n_kinects);
	openni::VideoStream** color_stream = (openni::VideoStream**)malloc(sizeof(openni::VideoStream*) * n_kinects);
	openni::VideoFrameRef** depth_frame = (openni::VideoFrameRef**)malloc(sizeof(openni::VideoFrameRef*) * n_kinects);
	openni::VideoFrameRef** color_frame = (openni::VideoFrameRef**)malloc(sizeof(openni::VideoFrameRef*) * n_kinects);

	for(int i = 0 ; i < n_kinects ; ++i){
		openni::VideoStream& depth_stream_temp(*depth[i]);
		depth_stream[i] = &depth_stream_temp;
		depth_frame[i] = new openni::VideoFrameRef();

		if(flag_color){
			openni::VideoStream& color_stream_temp(*color[i]);
			color_stream[i] = &color_stream_temp;
			//color_stream[i]->setMirroringEnabled(true);

			color_frame[i] = new openni::VideoFrameRef();
		}
	}

	openni::VideoStream** m_streams = new openni::VideoStream*[n_kinects * color_step];
	for(int i = 0 ; i < n_kinects; i++){
		m_streams[i] = depth_stream[i];

		if(flag_color){
			m_streams[n_kinects + i] = color_stream[i];
		}
	}
	
	
	cv::Mat** mat_depth = (cv::Mat**)malloc(sizeof(cv::Mat*) * n_kinects);
	cv::Mat** mat_color = (cv::Mat**)malloc(sizeof(cv::Mat*) * n_kinects);
	cv::Mat** mat_user = (cv::Mat**)malloc(sizeof(cv::Mat*) * n_kinects);

	for(int i = 0 ; i < n_kinects ; i++){
		mat_depth[i] = new cv::Mat(640,480,CV_8UC1);
		mat_color[i] = new cv::Mat(640,480,CV_8UC3);
		mat_user[i] = new cv::Mat(640,480,CV_8UC1);
	}

	nite::NiTE::initialize();
	nite::UserTracker** user_trackers = (nite::UserTracker**)malloc(sizeof(nite::UserTracker*) * n_kinects);
	nite::UserTrackerFrameRef** user_frames = (nite::UserTrackerFrameRef**)malloc(sizeof(nite::UserTrackerFrameRef*) * n_kinects);
	nite::UserMap** user_maps = (nite::UserMap**)malloc(sizeof(nite::UserMap*) * n_kinects);

	for(int i = 0 ; i < n_kinects ; i++){
		user_trackers[i] = new nite::UserTracker;
		user_frames[i] = new nite::UserTrackerFrameRef();
		user_maps[i] = new nite::UserMap();

		if (user_trackers[i]->create(devices[i]) != nite::STATUS_OK)	{
			return openni::STATUS_ERROR;
		}
	}

	char c = 0;
	while((c = cv::waitKey(12)) != 27){
		//****************************************************************
		//* UPDATE OPENNI
		//****************************************************************
		int changedIndex;
		openni::Status rc = openni::OpenNI::waitForAnyStream(m_streams, n_kinects * color_step, &changedIndex);
		if (rc != openni::STATUS_OK)
		{
			printf("Wait failed\n");
			return 3;
		}

		for(int i = 0 ; i < n_kinects ; i++){
			nite::Status rc_nite = user_trackers[i]->readFrame(user_frames[i]);
			if (rc_nite != nite::STATUS_OK){
				printf("GetNextData failed\n");
				return 4;
			}
						
			const nite::UserMap& userLabels = user_frames[i]->getUserMap();
			//user_maps[i] = (nite::UserMap*)&userLabels;

			cv::Mat depth_nite_16(480,640,CV_16UC1,(void*)userLabels.getPixels());
			cv::Mat depth_nite_8;
			depth_nite_16.convertTo(depth_nite_8, CV_8UC1);
			depth_nite_8.assignTo(*mat_user[i]);
			//cv::erode(*mat_user[i],*mat_user[i],cv::Mat(5,1,CV_8UC1));
		}

		for(int i = 0 ; i < n_kinects ; i++){
			rc = depth_stream[i]->readFrame(depth_frame[i]);
			if(flag_color){
				rc = color_stream[i]->readFrame(color_frame[i]);
			}
		}

		for(int i = 0 ; i < n_kinects ; i++){
			cv::Mat depthMat16UC1(480,640,CV_16UC1,(void*)depth_frame[i]->getData());
			cv::Mat depthMat8UC1;
			depthMat16UC1.convertTo(*mat_depth[i], CV_8UC1,0.05);
					
			if(flag_color){
				cv::Mat color_aux1(480,640,CV_8UC3,(void*) color_frame[i]->getData());
				cv::Mat color1,color2;
				cv::cvtColor(color_aux1,color1,CV_RGB2BGR);
				color1.copyTo(color2,*mat_user[i]);
				color2.assignTo(*mat_color[i]);
			}
		}

		for(int i = 0 ; i < n_kinects ; i++){
			char win_depth[128];
			sprintf(win_depth,"Depth (%d)",i);
			
			cv::imshow(win_depth,*mat_depth[i]);

			if(flag_color){
				char win_color[128];
				sprintf(win_color,"Color (%d)",i);
			
				cv::imshow(win_color,*mat_color[i]);
			}
		}

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

	return 0;
}