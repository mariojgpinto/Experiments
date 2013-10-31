#include "SimpleViewer.h"

#include <OpenNI.h>
#include <NiTE.h>
#include <opencv2\opencv.hpp>

class Kinect2{
public:

enum GENERATORS{
	DEPTH = 0,
	COLOR = 1
};

Kinect2(char* uri = NULL){
	_flags[DEPTH] = false;
	_flags[COLOR] = false;

	m_streams = new openni::VideoStream*[2];

	//m_streams[0] = &depth;
	//m_streams[1] = &color;
	

	const char* deviceURI;

	if(uri){
		deviceURI = uri;
	}
	else{
		deviceURI = openni::ANY_DEVICE;
	}

	openni::Status rc = openni::OpenNI::initialize();

	printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());
	
	rc = device.open(deviceURI);

	if (rc != openni::STATUS_OK)
	{
		printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		getchar();
		openni::OpenNI::shutdown();
		exit (0);
	}
}

bool activate_depth(){
	if(_flags[DEPTH]) return true;

	openni::Status rc = depth.create(device, openni::SENSOR_DEPTH);

	if (rc == openni::STATUS_OK)	{
		rc = depth.start();
		if (rc != openni::STATUS_OK){
			printf("Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			depth.destroy();
			return false;
		}

		if (!depth.isValid()){
			printf("No valid depth stream. Exiting\n");
			//openni::OpenNI::shutdown();
			return false;
		}
		_flags[DEPTH] = true;
		return true;
	}
	else{
		printf("Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
		return false;
	}
}

bool disable_depth(){
	if(!_flags[DEPTH]) return true;

	depth.stop();
	m_depthFrame.release();

	_flags[DEPTH] = false;

	return true;
}

bool activate_color(){
	if(_flags[COLOR]) return true;

	openni::Status rc = color.create(device, openni::SENSOR_COLOR);

	const openni::Array<openni::VideoMode> &modes = device.getSensorInfo(openni::SensorType::SENSOR_COLOR)->getSupportedVideoModes();

	openni::VideoMode vid0 = modes[0];
	openni::VideoMode vid1 = modes[1];
	openni::VideoMode vid2 = modes[2];

	openni::VideoMode vid;
	vid.setFps(12);
	vid.setResolution(1280,960);
	vid.setPixelFormat(openni::PixelFormat(ONI_PIXEL_FORMAT_RGB888));
	rc = color.setVideoMode(vid);

	if (rc == openni::STATUS_OK)	{
		rc = color.start();
		if (rc != openni::STATUS_OK){
			printf("Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
			color.destroy();
			return false;
		}

		if (!color.isValid()){
			printf("No valid color stream. Exiting\n");
			//openni::OpenNI::shutdown();
			return false;
		}
		_flags[COLOR] = true;
		return true;
	}
	else{
		printf("Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
		return false;
	}
}

bool disable_color(){
	if(!_flags[COLOR]) return true;

	color.stop();
	m_colorFrame.release();
	color.destroy();

	_flags[COLOR] = false;

	return true;
}

bool update(){
	if(_flags[Kinect2::DEPTH] || _flags[Kinect2::COLOR] ){
		int changedIndex;
		openni::Status rc = openni::OpenNI::waitForAnyStream(m_streams, 2, &changedIndex);
		if (rc != openni::STATUS_OK){
			printf("Wait failed\n");
			return false;
		}
		return true;
	}
	return true;
}

void close(){
	disable_color();
	disable_depth();

	//depth.destroy();
	//color.destroy();

	openni::OpenNI::shutdown();
}

int _flags[2];

openni::Device device;
openni::VideoStream depth, color;

openni::VideoFrameRef		m_depthFrame;
openni::VideoFrameRef		m_colorFrame;

openni::VideoStream**		m_streams;
};	




int main_simple_viewer(int argc, char* argv[]){
	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;
	bool result = false;
	openni::Status rc = openni::STATUS_OK;

	Kinect2 kinect;
		
	//result = kinect.activate_depth();
	//result = kinect.activate_color();

	//rc = kinect.device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );
	rc = kinect.device.setDepthColorSyncEnabled(true);
	
	kinect.m_streams[0] = &kinect.depth;
	kinect.m_streams[1] = &kinect.color;
	
	cv::Mat depthMat8UC1;

	bool regist = false;
	
	nite::NiTE::initialize();
	nite::UserTracker* m_pUserTracker = new nite::UserTracker;

	if (m_pUserTracker->create(&kinect.device) != nite::STATUS_OK)	{
		return openni::STATUS_ERROR;
	}

	nite::SkeletonState g_skeletonStates[10] = {nite::SKELETON_NONE};

	nite::UserTrackerFrameRef userTrackerFrame;
	openni::VideoFrameRef depthFrame;
	openni::VideoFrameRef depthFrame_nite;
	
	nite::UserMap user_map;
	
	cv::namedWindow("Depth");
	char c = 0;
	while((c = cv::waitKey(12)) != 27){
		if(!kinect.update())
			break;

		nite::Status rc_nite = m_pUserTracker->readFrame(&userTrackerFrame);
		if (rc_nite != nite::STATUS_OK){
			printf("GetNextData failed\n");
			return 3;
		}

		const nite::UserMap& userLabels = userTrackerFrame.getUserMap();
		

		if(kinect._flags[Kinect2::DEPTH]){
			rc = kinect.depth.readFrame(&kinect.m_depthFrame);

			cv::Mat depthMat16UC1(480,640,CV_16UC1,(void*)kinect.m_depthFrame.getData());
			depthMat16UC1.convertTo(depthMat8UC1, CV_8UC1,0.05);
			cv::imshow("Depth",depthMat8UC1);
		}

		if(kinect._flags[Kinect2::COLOR]){
			rc = kinect.color.readFrame(&kinect.m_colorFrame);

			cv::Mat color(960,1280,CV_8UC3,(void*) kinect.m_colorFrame.getData());
			cv::Mat color2;
			cv::cvtColor(color,color2,CV_RGB2BGR);
			cv::Mat color3;
			if(kinect._flags[Kinect2::DEPTH]){
				color2.copyTo(color3,depthMat8UC1);
				cv::imshow("color",color3);
			}
			else{
				cv::imshow("color",color2);
			}			
		}
		
		if(c == 'c'){
			if(kinect._flags[Kinect2::COLOR]) kinect.disable_color();
			else kinect.activate_color();
		}
		if(c == 'd'){
			if(kinect._flags[Kinect2::DEPTH]) kinect.disable_depth();
			else kinect.activate_depth();
		}
		if(c == 'i'){
			rc = openni::OpenNI::initialize();
		}
		if(c == 'r'){
			if(regist){
				rc = kinect.device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
			}
			else{
				rc = kinect.device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );
			}
			regist = !regist;
		}
		if(c == 'f'){
			//openni::VideoStream::
			nite::Plane plane = userTrackerFrame.getFloor();
			//nite::UserTrackerFrameRef::getFloor();
			printf("");
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

	userTrackerFrame.release();
	m_pUserTracker->destroy();
	nite::NiTE::shutdown();

	kinect.close();	
	
	return 0;
}