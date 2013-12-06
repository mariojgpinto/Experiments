#include "MultiViewer.h"

#include <OpenNI.h>
#include <opencv2\opencv.hpp>

const bool flag_color = true;

int main_multi_viewer(int argc, char* argv[]){	
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
			depth_stream[i]->setMirroringEnabled(true);

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

	for(int i = 0 ; i < n_kinects ; i++){
		mat_depth[i] = new cv::Mat(640,480,CV_8UC1);
		mat_color[i] = new cv::Mat(640,480,CV_8UC3);
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
				cv::Mat color1,color2,color3;
				cv::cvtColor(color_aux1,color1,CV_RGB2BGR);
				
				//cv::blur(color1,color2,cv::Size(3,3));
				//cv::blur(color1,color3,cv::Size(5,5));

				//cv::subtract(color3,color2,*mat_color[i]);

				cv::vector<cv::Mat> channels; 
				//cv::Mat img_hist_equalized;

				//cv::cvtColor(color1, img_hist_equalized, CV_BGR2YCrCb); //change the color image from BGR to YCrCb format

				cv::split(color1,channels); //split the image into channels

				cv::normalize(channels[0], channels[0],0,255,cv::NORM_MINMAX); //equalize histogram on the 1st channel (Y)
				cv::normalize(channels[1], channels[1],0,255,cv::NORM_MINMAX); //equalize histogram on the 1st channel (Y)
				cv::normalize(channels[2], channels[2],0,255,cv::NORM_MINMAX); //equalize histogram on the 1st channel (Y)

				cv::merge(channels,*mat_color[i]); //merge 3 channels including the modified 1st channel into one image

				//cv::cvtColor(img_hist_equalized, *mat_color[i], CV_YCrCb2BGR); //change the color image from YCrCb to BGR format (to display image properly)

				
				//cv::equalizeHist(color1,*mat_color[i]);
				//color1.copyTo(color2,*mat_depth[i]);
				//color2.assignTo(*mat_color[i]);
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