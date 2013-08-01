#include <_ID.h>

#include "multi_NIKinect.h"
#include <NIKinect.h>


XnCallbackHandle _openni_callback_user_handle;

void XN_CALLBACK_TYPE openni_callback_user_new(xn::UserGenerator& generator, XnUserID nId, void* pCookie){
	//UserManager *instance = (UserManager*)pCookie;
	printf("New User\n");
	//instance->add_openni_event(nId,UserManager::NEW_USER);
}

void XN_CALLBACK_TYPE openni_callback_user_lost(xn::UserGenerator& generator, XnUserID nId, void* pCookie){
	//UserManager *instance = (UserManager*)pCookie;
	printf("User Lost\n");
	//instance->add_openni_event(nId,UserManager::LOST_USER);
}



int main_multi_ni_kinect(int argc, char* argv[]){
	NIKinect* kinect1 = new NIKinect();
	NIKinect* kinect2 = new NIKinect();
	bool result = false;


#ifdef _CCG
	//kinect2->set_image_mode(1280,1024,15);
	result = kinect2->init(0,0);//,3+NIKinect::SCENE_A);//,3+NIKinect::USER_G);//,3+NIKinect::SCENE_A);//
	result = kinect1->init(0,1,3+NIKinect::USER_G+NIKinect::SCENE_A);//,3+NIKinect::SCENE_A);//);	

	xn::UserGenerator _user_generator = kinect1 ->get_user_generator();

	XnStatus s = _user_generator.RegisterUserCallbacks(	openni_callback_user_new,
											openni_callback_user_lost,
											NULL,
											_openni_callback_user_handle);
#endif

#ifdef _HOME
	//result = kinect1->init("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni", NIKinect::DEPTH_G + NIKinect::IMAGE_G + NIKinect::SCENE_A + NIKinect::USER_G);
	result = kinect1->init(0,0);
	result = kinect2->init(0,1);
#endif



	//xn::SceneAnalyzer xn_scene1;
	//xn::SceneAnalyzer xn_scene2;

	cv::Mat color_1;
	cv::Mat color_2;
	cv::Mat depth_1;
	cv::Mat depth_2;
	cv::Mat mask_1;
	cv::Mat mask_2;
	cv::Mat depthMat8UC1_1;
	cv::Mat depthMat8UC1_2;
	cv::Mat3b depth_as_color;

	int _min_bar = 400;
	int _max_bar = 1500;

	//cv::namedWindow("DepthAsColor");
	//cv::createTrackbar("MinDepth", "DepthAsColor", &_min_bar, 5000, NULL);
	//cv::createTrackbar("MaxDepth", "DepthAsColor", &_max_bar, 5000, NULL);

	char c = 0;
	while((c = cv::waitKey(31)) != 27){
		if(!kinect1->update()) 
			break;
		if(kinect1->get_color(color_1)){
			//kinect1->get_mask(mask_1);
			//cv::Mat color;

			//color_1.copyTo(color,mask_1);
			imshow("Color1",color_1);
		}
		//if(kinect1->get_depth(depth_1)){
		//	depth_1.convertTo(depthMat8UC1_1, CV_8UC1,0.05);
		//	imshow("Depth1",depthMat8UC1_1);
		//}



		if(!kinect2->update()) 
			break;
		if(kinect2->get_color(color_2)){
			//kinect2->get_mask(mask_2);
			//cv::Mat color;

			//color_2.copyTo(color,mask_2);
			imshow("Color2",color_2	);
		}
		//if(kinect2->get_depth(depth_2)){
		//	depth_2.convertTo(depthMat8UC1_2, CV_8UC1,0.05);
		//	imshow("Depth2",depthMat8UC1_2);
		//}

		

		if(c == '1'){
			double a,b,c,d;

			if(kinect1->get_floor_plane(&a,&b,&c,&d)){
				printf("Floor 1 (%.4f  %.4f  %.4f  %.4f)\n",a,b,c,d);
			}
			else{
				printf("No Floor 1\n");
			}
			//XnPlane3D floorCoords;
			//if(XN_STATUS_OK == kinect1->get_scene_analyzer().GetFloor(floorCoords)){
			//	printf("Floor 1\n");
			//}
		}

		if(c == '2'){
			double a,b,c,d;

			if(kinect2->get_floor_plane(&a,&b,&c,&d)){
				printf("Floor 2 (%.4f  %.4f  %.4f  %.4f)\n",a,b,c,d);
			}
			else{
				printf("No Floor 2\n");
			}
			//XnPlane3D floorCoords;
			//if(XN_STATUS_OK == kinect2->get_scene_analyzer().GetFloor(floorCoords)){
			//	printf("Floor 2\n");
			//}
		}

		printf("Kinect1 (%.2f)    Kinect2 (%.2f)\n",kinect1->get_frame_rate(), kinect2->get_frame_rate());
	}


	return 0;
}