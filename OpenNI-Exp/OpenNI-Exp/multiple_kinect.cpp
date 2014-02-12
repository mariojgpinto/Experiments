#include "multiple_kinect.h"

#include "RemoveFloor.h"

#include <XnOS.h>
#include <math.h>

#include <XnCppWrapper.h>
using namespace xn;

#include <opencv2\opencv.hpp>

#define SAMPLE_XML_PATH "C:\\Dev\\External\\OpenNI\\Data\\SamplesConfig.xml"
void compute_color_encoded_depth(const cv::Mat1f& depth_im, cv::Mat3b& color_depth_im,
                                     double* i_min_val, double* i_max_val);

Context							_g_context;

std::vector<DepthGenerator*>	_g_depth;
std::vector<ImageGenerator*>	_g_image;
std::vector<DepthMetaData*>		_g_depthMD;
std::vector<ImageMetaData*>		_g_imageMD;
std::vector<UserGenerator*>		_g_user;

int								_nKinects;


#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
	}	


void XN_CALLBACK_TYPE NewUser_0(UserGenerator& generator, XnUserID user, void* pCookie){
	printf("New User 0 - ID %d\n", user);
		
	//_g_user[0]->GetSkeletonCap().StartTracking(user);
	return;
}
void XN_CALLBACK_TYPE NewUser_1(UserGenerator& generator, XnUserID user, void* pCookie){
	printf("New User 1 - ID %d\n",user);
		
	//_g_user[1]->GetSkeletonCap().StartTracking(user);
	return;
}

void XN_CALLBACK_TYPE LostUser_0(UserGenerator& generator, XnUserID user, void* pCookie){
	printf("Lost user 0\n");
}
void XN_CALLBACK_TYPE LostUser_1(UserGenerator& generator, XnUserID user, void* pCookie){
	printf("Lost user 1\n");
}


//void XN_CALLBACK_TYPE PoseDetected(PoseDetectionCapability& pose, const XnChar* strPose, XnUserID user, void* cxt)
//{
//	printf("Found pose \"%s\" for user %d\n", strPose, user);
//	_g_user->GetSkeletonCap().RequestCalibration(user, TRUE);
//	_g_user->GetPoseDetectionCap().StopPoseDetection(user);
//}
//
//void XN_CALLBACK_TYPE CalibrationStarted(SkeletonCapability& skeleton, XnUserID user, void* cxt){
//	printf("Calibration started\n");
//}
//
///**
// * @brief	OpenNI Callback when a user's calibration process is completed.
// */
//void XN_CALLBACK_TYPE CalibrationCompleted(SkeletonCapability& skeleton, XnUserID user, XnCalibrationStatus eStatus, void* cxt){
//	printf("Calibration done [%d] %ssuccessfully\n", user, (eStatus == XN_CALIBRATION_STATUS_OK)?"":"un");
//	if (eStatus == XN_CALIBRATION_STATUS_OK){
//		if (!g_bCalibrated)	{
//			_g_user->GetSkeletonCap().SaveCalibrationData(user, 0);
//			g_nPlayer = user;
//			_g_user->GetSkeletonCap().StartTracking(user);
//			g_bCalibrated = TRUE;
//		}
//
//		XnUserID aUsers[10];
//		for(int i = 0 ; i < 10 ; ++i){
//			aUsers[i] = 99;
//		}
//		XnUInt16 nUsers = 10;
//		_g_user->GetUsers(aUsers, nUsers);
//		for (int i = 0; i < nUsers; ++i)
//			_g_user->GetPoseDetectionCap().StopPoseDetection(aUsers[i]);
//	}
//}


int _mk_init() {

	XnStatus status;
	
	status = _g_context.Init();
	//status = _g_context[1].Init();

	static xn::NodeInfoList node_info_list;
	static xn::NodeInfoList depth_nodes;
	static xn::NodeInfoList image_nodes;
	static xn::NodeInfoList user_nodes;

	status = _g_context.EnumerateProductionTrees (XN_NODE_TYPE_DEVICE, NULL, node_info_list);

	if (status != XN_STATUS_OK && node_info_list.Begin () != node_info_list.End ()) { 
		printf ("Enumerating devices failed. Reason: %s", xnGetStatusString (status)); 
		return -1;
	}
	else if (node_info_list.Begin () == node_info_list.End ()) {
		printf("No devices found.\n");
		return -1;
	}

	for (xn::NodeInfoList::Iterator nodeIt = node_info_list.Begin (); nodeIt != node_info_list.End (); ++nodeIt) {
		_nKinects++;
		xn::NodeInfo info = *nodeIt;
		const XnProductionNodeDescription& description = info.GetDescription();
		printf("image: vendor (%s) name (%s), \ninstance (%s), \ndescription (%s)\n\n\n",	description.strVendor, 
																				description.strName, 
																				info.GetInstanceName(),
																				info.GetCreationInfo());
	}

	status = _g_context.EnumerateProductionTrees (XN_NODE_TYPE_IMAGE, NULL, image_nodes, NULL);

	if (status != XN_STATUS_OK && image_nodes.Begin () != image_nodes.End ()) { 
		printf ("Enumerating devices failed. Reason: %s", xnGetStatusString (status)); 
		return -1;
	}
	else if (image_nodes.Begin () == image_nodes.End ()) {
		printf("No devices found.\n");
		return -1;
	}

	status = _g_context.EnumerateProductionTrees (XN_NODE_TYPE_DEPTH, NULL, depth_nodes, NULL);

	if (status != XN_STATUS_OK && depth_nodes.Begin () != depth_nodes.End ()) { 
		printf ("Enumerating devices failed. Reason: %s", xnGetStatusString (status)); 
		return -1;
	}
	else if (depth_nodes.Begin () == depth_nodes.End ()) {
		printf("No devices found.\n");
		return -1;
	}

	status = _g_context.EnumerateProductionTrees (XN_NODE_TYPE_USER, NULL, user_nodes, NULL);

	if (status != XN_STATUS_OK && user_nodes.Begin () != user_nodes.End ()) { 
		printf ("Enumerating devices failed. Reason: %s", xnGetStatusString (status)); 
		return -1;
	}
	else if (user_nodes.Begin () == user_nodes.End ()) {
		printf("No devices found.\n");
		return -1;
	}

	int i = 0;

	

	i = 0;
	//Image Generators
	for (xn::NodeInfoList::Iterator nodeIt =image_nodes.Begin(); nodeIt != image_nodes.End(); ++nodeIt, i++) {
		xn::NodeInfo info = *nodeIt;

		XnMapOutputMode mode;
		mode.nXRes	= 640;
		mode.nYRes	= 480;
		mode.nFPS	= 30;

		status = _g_context.CreateProductionTree (info);

		ImageGenerator* g_image = new ImageGenerator();
		ImageMetaData* g_imageMD = new ImageMetaData();

		status = info.GetInstance (*g_image);

		status = g_image->SetMapOutputMode(mode);
		g_image->GetMetaData(*g_imageMD);
		g_image->StartGenerating();

		_g_image.push_back(g_image);
		_g_imageMD.push_back(g_imageMD);
	}

	i = 0;
	//Depth Generators
	for (xn::NodeInfoList::Iterator nodeIt =depth_nodes.Begin(); nodeIt != depth_nodes.End(); ++nodeIt, i++) {
		xn::NodeInfo info = *nodeIt;

		XnMapOutputMode mode;
		mode.nXRes	= 640;
		mode.nYRes	= 480;
		mode.nFPS	= 30;

		status = _g_context.CreateProductionTree (info);

		DepthGenerator* g_depth = new DepthGenerator();
		DepthMetaData* g_depthMD = new DepthMetaData();

		status = info.GetInstance (*g_depth);

		g_depth->SetMapOutputMode(mode);
		g_depth->GetMetaData(*g_depthMD);
		g_depth->StartGenerating();

		_g_depth.push_back(g_depth);
		_g_depthMD.push_back(g_depthMD);
	}

	i = 0;

	//User Generators
	//xn::NodeInfoList list;
	//std::string info_str("user nodes");
	//status = _g_context.EnumerateProductionTrees (XN_NODE_TYPE_USER, NULL, list, NULL);

	//if (status != XN_STATUS_OK && list.Begin () != list.End ()) { 
	//	printf ("Enumerating %s failed. Reason: %s", info_str, xnGetStatusString (status)); 
	//	return -1;
	//}
	//else if (list.Begin () == list.End ()) {
	//	printf("No %s found.\n", info_str);
	//	return -1;
	//}

	int numNodes = 0;
	for (xn::NodeInfoList::Iterator nodeIt = user_nodes.Begin (); nodeIt != user_nodes.End (); ++nodeIt, numNodes++) {
		const xn::NodeInfo& info = *nodeIt;
		const XnProductionNodeDescription& description = info.GetDescription();
		printf("device %d vendor %s name %s, instance %s\n", numNodes, description.strVendor, description.strName, info.GetInstanceName());
	}

	//std::cout << "Finishing enumerating: " << info_str << std::endl;

	xn::NodeInfoList::Iterator nodeItUser  = user_nodes.Begin();
	xn::NodeInfo info = *nodeItUser;

	_g_context.CreateProductionTree(info);

	UserGenerator* g_user0= new UserGenerator();
	
	status = info.GetInstance(*g_user0);
	if (!g_user0->IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
		!g_user0->IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
		{
			printf("User generator 0 doesn't support either skeleton or pose detection.\n");
			return 0;
		}
	_g_user.push_back(g_user0);
	
	XnStatus rc;
	XnCallbackHandle hUserCBs, hCalibrationStartCB, hCalibrationCompleteCB, hPoseCBs;
	rc = g_user0->RegisterUserCallbacks(NewUser_0, LostUser_0, NULL, hUserCBs);
	CHECK_RC(rc,"Register 0\n");
	//Second user generator:

	nodeItUser++;nodeItUser++;nodeItUser++;nodeItUser++;nodeItUser++;//select the fourth node
	nodeItUser++;nodeItUser++;nodeItUser++;nodeItUser++;nodeItUser++;
	nodeItUser++;nodeItUser++;nodeItUser++;nodeItUser++;nodeItUser++;
	nodeItUser++;nodeItUser++;nodeItUser++;nodeItUser++;nodeItUser++;
	nodeItUser++;nodeItUser++;nodeItUser++;nodeItUser++;nodeItUser++;

	info = *nodeItUser;
	
	UserGenerator* g_user1= new UserGenerator();

	_g_context.CreateProductionTree(info);
	status = info.GetInstance(*g_user1);
	if (!g_user1->IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
		!g_user1->IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
		{
			printf("User generator 1 doesn't support either skeleton or pose detection.\n");
			return 0;
		}

	_g_user.push_back(g_user1);


	rc = g_user1->RegisterUserCallbacks(NewUser_1, LostUser_1, NULL, hUserCBs);
	CHECK_RC(rc,"Register 1\n");


	//for (xn::NodeInfoList::Iterator nodeIt =user_nodes.Begin(); nodeIt != user_nodes.End(); ++nodeIt, i++) {
	//	if(/*i == 0 || */i == 6) {
	//		xn::NodeInfo info = *nodeIt;
	//		printf("user %d\n",i);

	//		status = _g_context.CreateProductionTree (info);

	//		UserGenerator* g_user= new UserGenerator();

	//		status = info.GetInstance (*g_user);
	//		CHECK_RC(status,"User get instance\n");
	//	
	//		if (!g_user->IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
	//			!g_user->IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
	//		{
	//			printf("User generator doesn't support either skeleton or pose detection.\n");
	//			return 0;
	//		}
	//		g_user->GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	//		status = g_user->StartGenerating();
	//		CHECK_RC(status,"User start generating\n");
	//	
	//		_g_user.push_back(g_user);
	//	
	//		XnStatus rc;
	//		XnCallbackHandle hUserCBs, hCalibrationStartCB, hCalibrationCompleteCB, hPoseCBs;
	//		if(i == 0){
	//			rc = g_user->RegisterUserCallbacks(NewUser_0, LostUser_0, NULL, hUserCBs);
	//			CHECK_RC(rc,"Register 0\n");
	//		}
	//		else{
	//			rc = g_user->RegisterUserCallbacks(NewUser_1, LostUser_1, NULL, hUserCBs);
	//			CHECK_RC(rc,"Register 1\n");
	//		}
	//	}
	//}

	for (int i = 0; i < _nKinects; i++) {
		_g_image[i]->GetMirrorCap().SetMirror(false);
		_g_depth[i]->GetAlternativeViewPointCap().SetViewPoint(*_g_image[i]);
		_g_depth[i]->GetMirrorCap().SetMirror(false);
	}
 
	status =_g_context.StartGeneratingAll();
	
	return 1;

	//xn::NodeInfoList list;
	//std::string info_str("user nodes");
	//status = _g_context.EnumerateProductionTrees (XN_NODE_TYPE_USER, NULL, list, NULL);

	//if (status != XN_STATUS_OK && list.Begin () != list.End ()) { 
	//	printf ("Enumerating %s failed. Reason: %s", info_str, xnGetStatusString (status)); 
	//	return -1;
	//}
	//else if (list.Begin () == list.End ()) {
	//	printf("No %s found.\n", info_str);
	//	return -1;
	//}

	//int numNodes = 0;
	//for (xn::NodeInfoList::Iterator nodeIt = list.Begin (); nodeIt != list.End (); ++nodeIt, numNodes++) {
	//	const xn::NodeInfo& info = *nodeIt;
	//	const XnProductionNodeDescription& description = info.GetDescription();
	//	printf("device %d vendor %s name %s, instance %s\n", numNodes, description.strVendor, description.strName, info.GetInstanceName());
	//}

	//std::cout << "Finishing enumerating: " << info_str << std::endl;

	//xn::NodeInfoList::Iterator nodeItUser  = list.Begin();
	//xn::NodeInfo info = *nodeItUser;

	//_g_context.CreateProductionTree(info);

	//UserGenerator* g_user0= new UserGenerator();
	//UserGenerator* g_user1= new UserGenerator();

	//status = info.GetInstance(*g_user0);
	//if (!g_user0->IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
	//		!g_user0->IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
	//	{
	//		printf("User generator doesn't support either skeleton or pose detection.\n");
	//		return 0;
	//	}
	//_g_user.push_back(g_user0);
	//
	//XnStatus rc;
	//XnCallbackHandle hUserCBs, hCalibrationStartCB, hCalibrationCompleteCB, hPoseCBs;
	//rc = g_user0->RegisterUserCallbacks(NewUser_0, LostUser_0, NULL, hUserCBs);
	//CHECK_RC(rc,"Register 0\n");
	////Second user generator:

	//nodeItUser++;//nodeItUser++,nodeItUser++,nodeItUser++,nodeItUser++; //select the fourth node
	//info = *nodeItUser;
	//
	//_g_context.CreateProductionTree(info);
	//status = info.GetInstance(*g_user1);
	//if (!g_user1->IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
	//		!g_user1->IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
	//	{
	//		printf("User generator doesn't support either skeleton or pose detection.\n");
	//		return 0;
	//	}

	//_g_user.push_back(g_user1);


	//rc = g_user1->RegisterUserCallbacks(NewUser_1, LostUser_1, NULL, hUserCBs);
	//CHECK_RC(rc,"Register 1\n");
}

void _mk_update() {
	XnStatus status = XN_STATUS_OK;
	status = _g_context.WaitAndUpdateAll();

	if (status != XN_STATUS_OK) {
		printf("Read failed: %s\n", xnGetStatusString(status));
		return;
	}

	for (int i = 0; i<_nKinects; i++) {
		_g_image[i]->GetMetaData(*_g_imageMD[i]);
		_g_depth[i]->GetMetaData(*_g_depthMD[i]);

	}
}

int main_multiple_kinect(int argc, char* argv[]){
	//_g_context.push_back(*(new Context));
	//_g_context.push_back(*(new Context));
	_mk_init();
	//_mk_init_1();

	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

	XnStatus rc;

	int* _min_bar = (int*)malloc(sizeof(int)*_nKinects);// = 400;
	int* _max_bar = (int*)malloc(sizeof(int)*_nKinects);// = 1500;

	for (int i = 0; i<_nKinects; i++) {
		char buff[100];
		sprintf_s(buff,"Ranged Image (%d)",i);
		cv::namedWindow(buff);
		_min_bar[i] = 400;
		_max_bar[i] = 3500;
		cv::createTrackbar("MinDepth", buff, &_min_bar[i], 15000, NULL);
		cv::createTrackbar("MaxDepth", buff, &_max_bar[i], 15000, NULL);

	}
	

	XnUserID _openni_users_id[16];
	XnUInt16 temp_counter = 0;
	char c = 0;
	while((c = cv::waitKey(23)) != 27){
		// Read a new frame
		_mk_update();

		//Release a Kinect
		if(c == '1'){
			_nKinects--;
			ImageGenerator* temp_ig = _g_image[0];
			ImageMetaData * temp_imd = _g_imageMD[0];
			_g_image[0] = _g_image[1];
			_g_imageMD[0] = _g_imageMD[1];
			_g_image.pop_back();
			_g_imageMD.pop_back();
			temp_ig->Release();
			//temp_imd->Rele
			
			DepthGenerator* temp_dg = _g_depth[0];
			DepthMetaData * temp_dmd = _g_depthMD[0];
			_g_depth[0] = _g_depth[1];
			_g_depthMD[0] = _g_depthMD[1];
			_g_depth.pop_back();
			_g_depthMD.pop_back();
			//temp_dg->Release();
			temp_dg->Release();
			
			
			
			
			//_g_depth[_nKinects]->Release();
			//_g_image[_nKinects]->Release();

		}

		//if(c == '2'
		
		for (int i = 0; i<_nKinects; i++) {
			cv::Mat depthMat16UC1(480, 640,CV_16UC1, (void*) _g_depthMD[i]->Data());
			cv::Mat color(480,640,CV_8UC3,(void*) _g_imageMD[i]->Data());
			cv::Mat color_rgb;
			cv::cvtColor(color,color_rgb,CV_RGB2BGR);
		
			//Retrieve information from the user generator
			_g_user[i]->GetUsers(_openni_users_id,temp_counter);

			if(temp_counter){
				printf("user at generator %d\n",i);
			}
			//XnUInt16 ;
			//XnUInt16 temp_counter;
			////Retrieve information from the user generator
			//_g_user[i]->GetUsers(_openni_users_id,temp_counter);


			//cv::Mat3b colorize;
			//double _min = _min_bar[i];
			//double _max = _max_bar[i];
			//compute_color_encoded_depth(depthMat16UC1,colorize,&_min,&_max);

			//cv::Mat mask_cv;			
			//cv::inRange(depthMat16UC1,_min_bar[i],_max_bar[i],mask_cv);

			//cv::Mat color_range;
			//color_rgb.copyTo(color_range,mask_cv);

			//char buff[100];
			//sprintf_s(buff,"Ranged Image (%d)",i);
			//cv::imshow(buff,colorize);
			cv::Mat depthMat8UC1;
			depthMat16UC1.convertTo(depthMat8UC1,CV_8UC1,0.05);
			char buff1[100];
			sprintf_s(buff1,"Depth (%d)",i);
			cv::imshow(buff1,depthMat8UC1);

			char buff2[100];
			sprintf_s(buff2,"Color (%d)",i);
			cv::imshow(buff2,color_rgb);
		}

		


		//cv::imshow("Ranged Image",colorize);
		//cv::imshow("Rcolori",colorize);

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

	return 0;
}


/*


int _mk_init_0() {

	XnStatus status;
	
	status = _g_context[0].Init();
	//status = _g_context[1].Init();

	static xn::NodeInfoList node_info_list;
	static xn::NodeInfoList depth_nodes;
	static xn::NodeInfoList image_nodes;
	static xn::NodeInfoList user_nodes;

	status = _g_context[0].EnumerateProductionTrees (XN_NODE_TYPE_DEVICE, NULL, node_info_list);

	if (status != XN_STATUS_OK && node_info_list.Begin () != node_info_list.End ()) { 
		printf ("Enumerating devices failed. Reason: %s", xnGetStatusString (status)); 
		return -1;
	}
	else if (node_info_list.Begin () == node_info_list.End ()) {
		printf("No devices found.\n");
		return -1;
	}

	for (xn::NodeInfoList::Iterator nodeIt = node_info_list.Begin (); nodeIt != node_info_list.End (); ++nodeIt) {
		_nKinects++;
		xn::NodeInfo info = *nodeIt;
		const XnProductionNodeDescription& description = info.GetDescription();
		printf("image: vendor (%s) name (%s), \ninstance (%s), \ndescription (%s)\n\n\n",	description.strVendor, 
																				description.strName, 
																				info.GetInstanceName(),
																				info.GetCreationInfo());
	}

	status = _g_context[0].EnumerateProductionTrees (XN_NODE_TYPE_IMAGE, NULL, image_nodes, NULL);

	if (status != XN_STATUS_OK && image_nodes.Begin () != image_nodes.End ()) { 
		printf ("Enumerating devices failed. Reason: %s", xnGetStatusString (status)); 
		return -1;
	}
	else if (image_nodes.Begin () == image_nodes.End ()) {
		printf("No devices found.\n");
		return -1;
	}

	status = _g_context[0].EnumerateProductionTrees (XN_NODE_TYPE_DEPTH, NULL, depth_nodes, NULL);

	if (status != XN_STATUS_OK && depth_nodes.Begin () != depth_nodes.End ()) { 
		printf ("Enumerating devices failed. Reason: %s", xnGetStatusString (status)); 
		return -1;
	}
	else if (depth_nodes.Begin () == depth_nodes.End ()) {
		printf("No devices found.\n");
		return -1;
	}

	status = _g_context[0].EnumerateProductionTrees (XN_NODE_TYPE_USER, NULL, user_nodes, NULL);

	if (status != XN_STATUS_OK && user_nodes.Begin () != user_nodes.End ()) { 
		printf ("Enumerating devices failed. Reason: %s", xnGetStatusString (status)); 
		return -1;
	}
	else if (user_nodes.Begin () == user_nodes.End ()) {
		printf("No devices found.\n");
		return -1;
	}

	int i = 0;

	

	i = 0;
	//Image Generators
	for (xn::NodeInfoList::Iterator nodeIt =image_nodes.Begin(); nodeIt != image_nodes.End(); ++nodeIt, i++) {
		xn::NodeInfo info = *nodeIt;

		XnMapOutputMode mode;
		mode.nXRes	= 640;
		mode.nYRes	= 480;
		mode.nFPS	= 30;

		status = _g_context[0].CreateProductionTree (info);

		ImageGenerator* g_image = new ImageGenerator();
		ImageMetaData* g_imageMD = new ImageMetaData();

		status = info.GetInstance (*g_image);

		status = g_image->SetMapOutputMode(mode);
		g_image->GetMetaData(*g_imageMD);
		g_image->StartGenerating();

		_g_image.push_back(g_image);
		_g_imageMD.push_back(g_imageMD);
	}

	i = 0;
	//Depth Generators
	for (xn::NodeInfoList::Iterator nodeIt =depth_nodes.Begin(); nodeIt != depth_nodes.End(); ++nodeIt, i++) {
		xn::NodeInfo info = *nodeIt;

		XnMapOutputMode mode;
		mode.nXRes	= 640;
		mode.nYRes	= 480;
		mode.nFPS	= 30;

		status = _g_context[0].CreateProductionTree (info);

		DepthGenerator* g_depth = new DepthGenerator();
		DepthMetaData* g_depthMD = new DepthMetaData();

		status = info.GetInstance (*g_depth);

		g_depth->SetMapOutputMode(mode);
		g_depth->GetMetaData(*g_depthMD);
		g_depth->StartGenerating();

		_g_depth.push_back(g_depth);
		_g_depthMD.push_back(g_depthMD);
	}

	i = 0;

	//User Generators
	for (xn::NodeInfoList::Iterator nodeIt =user_nodes.Begin(); nodeIt != user_nodes.End(); ++nodeIt, i++) {
		xn::NodeInfo info = *nodeIt;
		printf("user %d\n",i);

		if(i == 0){
			status = _g_context[0].CreateProductionTree (info);

			UserGenerator* g_user= new UserGenerator();

			status = info.GetInstance (*g_user);
			CHECK_RC(status,"User get instance\n");
		
			if (!g_user->IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
				!g_user->IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
			{
				printf("User generator doesn't support either skeleton or pose detection.\n");
				return 0;
			}
			g_user->GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

			status = g_user->StartGenerating();
			CHECK_RC(status,"User start generating\n");
		
			_g_user.push_back(g_user);
		
			XnStatus rc;
			XnCallbackHandle hUserCBs, hCalibrationStartCB, hCalibrationCompleteCB, hPoseCBs;
			if(i == 0){
				rc = g_user->RegisterUserCallbacks(NewUser_0, LostUser_0, NULL, hUserCBs);
				CHECK_RC(rc,"Register 0\n");
			}
			else{
				rc = g_user->RegisterUserCallbacks(NewUser_1, LostUser_1, NULL, hUserCBs);
				CHECK_RC(rc,"Register 1\n");
			}
		}
	}

	for (int i = 0; i < _nKinects; i++) {
		_g_image[i]->GetMirrorCap().SetMirror(false);
		_g_depth[i]->GetAlternativeViewPointCap().SetViewPoint(*_g_image[i]);
		_g_depth[i]->GetMirrorCap().SetMirror(false);
	}
 
	status =_g_context[0].StartGeneratingAll();
	
	return 1;

	
	//xn::NodeInfoList list;
	//std::string info_str("user nodes");
	//status = _g_context.EnumerateProductionTrees (XN_NODE_TYPE_USER, NULL, list, NULL);

	//if (status != XN_STATUS_OK && list.Begin () != list.End ()) { 
	//	printf ("Enumerating %s failed. Reason: %s", info_str, xnGetStatusString (status)); 
	//	return -1;
	//}
	//else if (list.Begin () == list.End ()) {
	//	printf("No %s found.\n", info_str);
	//	return -1;
	//}

	//int numNodes = 0;
	//for (xn::NodeInfoList::Iterator nodeIt = list.Begin (); nodeIt != list.End (); ++nodeIt, numNodes++) {
	//	const xn::NodeInfo& info = *nodeIt;
	//	const XnProductionNodeDescription& description = info.GetDescription();
	//	printf("device %d vendor %s name %s, instance %s\n", numNodes, description.strVendor, description.strName, info.GetInstanceName());
	//}

	//std::cout << "Finishing enumerating: " << info_str << std::endl;

	//xn::NodeInfoList::Iterator nodeItUser  = list.Begin();
	//xn::NodeInfo info = *nodeItUser;

	//_g_context.CreateProductionTree(info);

	//UserGenerator* g_user0= new UserGenerator();
	//UserGenerator* g_user1= new UserGenerator();

	//status = info.GetInstance(*g_user0);
	//if (!g_user0->IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
	//		!g_user0->IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
	//	{
	//		printf("User generator doesn't support either skeleton or pose detection.\n");
	//		return 0;
	//	}
	//_g_user.push_back(g_user0);
	//
	//XnStatus rc;
	//XnCallbackHandle hUserCBs, hCalibrationStartCB, hCalibrationCompleteCB, hPoseCBs;
	//rc = g_user0->RegisterUserCallbacks(NewUser_0, LostUser_0, NULL, hUserCBs);
	//CHECK_RC(rc,"Register 0\n");
	////Second user generator:

	//nodeItUser++;//nodeItUser++,nodeItUser++,nodeItUser++,nodeItUser++; //select the fourth node
	//info = *nodeItUser;
	//
	//_g_context.CreateProductionTree(info);
	//status = info.GetInstance(*g_user1);
	//if (!g_user1->IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
	//		!g_user1->IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
	//	{
	//		printf("User generator doesn't support either skeleton or pose detection.\n");
	//		return 0;
	//	}

	//_g_user.push_back(g_user1);


	//rc = g_user1->RegisterUserCallbacks(NewUser_1, LostUser_1, NULL, hUserCBs);
	//CHECK_RC(rc,"Register 1\n");
}


*/


/*
int _mk_init_0() {

	XnStatus status;
	
	status = _g_context[0].Init();
	//status = _g_context[1].Init();

	static xn::NodeInfoList node_info_list;
	static xn::NodeInfoList depth_nodes;
	static xn::NodeInfoList image_nodes;
	static xn::NodeInfoList user_nodes;

	status = _g_context[0].EnumerateProductionTrees (XN_NODE_TYPE_DEVICE, NULL, node_info_list);

	if (status != XN_STATUS_OK && node_info_list.Begin () != node_info_list.End ()) { 
		printf ("Enumerating devices failed. Reason: %s", xnGetStatusString (status)); 
		return -1;
	}
	else if (node_info_list.Begin () == node_info_list.End ()) {
		printf("No devices found.\n");
		return -1;
	}

	for (xn::NodeInfoList::Iterator nodeIt = node_info_list.Begin (); nodeIt != node_info_list.End (); ++nodeIt) {
		_nKinects++;
		xn::NodeInfo info = *nodeIt;
		const XnProductionNodeDescription& description = info.GetDescription();
		printf("image: vendor (%s) name (%s), \ninstance (%s), \ndescription (%s)\n\n\n",	description.strVendor, 
																				description.strName, 
																				info.GetInstanceName(),
																				info.GetCreationInfo());
	}

	status = _g_context[0].EnumerateProductionTrees (XN_NODE_TYPE_IMAGE, NULL, image_nodes, NULL);

	if (status != XN_STATUS_OK && image_nodes.Begin () != image_nodes.End ()) { 
		printf ("Enumerating devices failed. Reason: %s", xnGetStatusString (status)); 
		return -1;
	}
	else if (image_nodes.Begin () == image_nodes.End ()) {
		printf("No devices found.\n");
		return -1;
	}

	status = _g_context[0].EnumerateProductionTrees (XN_NODE_TYPE_DEPTH, NULL, depth_nodes, NULL);

	if (status != XN_STATUS_OK && depth_nodes.Begin () != depth_nodes.End ()) { 
		printf ("Enumerating devices failed. Reason: %s", xnGetStatusString (status)); 
		return -1;
	}
	else if (depth_nodes.Begin () == depth_nodes.End ()) {
		printf("No devices found.\n");
		return -1;
	}

	status = _g_context[0].EnumerateProductionTrees (XN_NODE_TYPE_USER, NULL, user_nodes, NULL);

	if (status != XN_STATUS_OK && user_nodes.Begin () != user_nodes.End ()) { 
		printf ("Enumerating devices failed. Reason: %s", xnGetStatusString (status)); 
		return -1;
	}
	else if (user_nodes.Begin () == user_nodes.End ()) {
		printf("No devices found.\n");
		return -1;
	}

	int i = 0;

	

	i = 0;
	//Image Generators
	for (xn::NodeInfoList::Iterator nodeIt =image_nodes.Begin(); nodeIt != image_nodes.End(); ++nodeIt, i++) {
		if(i == 0){
		xn::NodeInfo info = *nodeIt;

		XnMapOutputMode mode;
		mode.nXRes	= 640;
		mode.nYRes	= 480;
		mode.nFPS	= 30;

		status = _g_context[0].CreateProductionTree (info);

		ImageGenerator* g_image = new ImageGenerator();
		ImageMetaData* g_imageMD = new ImageMetaData();

		status = info.GetInstance (*g_image);

		status = g_image->SetMapOutputMode(mode);
		g_image->GetMetaData(*g_imageMD);
		g_image->StartGenerating();

		_g_image.push_back(g_image);
		_g_imageMD.push_back(g_imageMD);
		}
	}

	i = 0;
	//Depth Generators
	for (xn::NodeInfoList::Iterator nodeIt =depth_nodes.Begin(); nodeIt != depth_nodes.End(); ++nodeIt, i++) {
		if(i == 0){
		xn::NodeInfo info = *nodeIt;

		XnMapOutputMode mode;
		mode.nXRes	= 640;
		mode.nYRes	= 480;
		mode.nFPS	= 30;

		status = _g_context[0].CreateProductionTree (info);

		DepthGenerator* g_depth = new DepthGenerator();
		DepthMetaData* g_depthMD = new DepthMetaData();

		status = info.GetInstance (*g_depth);

		g_depth->SetMapOutputMode(mode);
		g_depth->GetMetaData(*g_depthMD);
		g_depth->StartGenerating();

		_g_depth.push_back(g_depth);
		_g_depthMD.push_back(g_depthMD);
		}
	}

	i = 0;

	//User Generators
	for (xn::NodeInfoList::Iterator nodeIt =user_nodes.Begin(); nodeIt != user_nodes.End(); ++nodeIt, i++) {
		if(i == 0){
		xn::NodeInfo info = *nodeIt;
		printf("user %d\n",i);

		
			status = _g_context[0].CreateProductionTree (info);

			UserGenerator* g_user= new UserGenerator();

			status = info.GetInstance (*g_user);
			CHECK_RC(status,"User get instance\n");
		
			if (!g_user->IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
				!g_user->IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
			{
				printf("User generator doesn't support either skeleton or pose detection.\n");
				return 0;
			}
			g_user->GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

			status = g_user->StartGenerating();
			CHECK_RC(status,"User start generating\n");
		
			_g_user.push_back(g_user);
		
			XnStatus rc;
			XnCallbackHandle hUserCBs, hCalibrationStartCB, hCalibrationCompleteCB, hPoseCBs;
			//if(i == 0){
				rc = g_user->RegisterUserCallbacks(NewUser_0, LostUser_0, NULL, hUserCBs);
				CHECK_RC(rc,"Register 0\n");
			//}
			//else{
			//	rc = g_user->RegisterUserCallbacks(NewUser_1, LostUser_1, NULL, hUserCBs);
			//	CHECK_RC(rc,"Register 1\n");
			//}
		}
	}

	//for (int i = 0; i < _nKinects; i++) {
		_g_image[0]->GetMirrorCap().SetMirror(false);
		_g_depth[0]->GetAlternativeViewPointCap().SetViewPoint(*_g_image[0]);
		_g_depth[0]->GetMirrorCap().SetMirror(false);
	//}
 
	status =_g_context[0].StartGeneratingAll();
	
	return 1;

	
	//xn::NodeInfoList list;
	//std::string info_str("user nodes");
	//status = _g_context.EnumerateProductionTrees (XN_NODE_TYPE_USER, NULL, list, NULL);

	//if (status != XN_STATUS_OK && list.Begin () != list.End ()) { 
	//	printf ("Enumerating %s failed. Reason: %s", info_str, xnGetStatusString (status)); 
	//	return -1;
	//}
	//else if (list.Begin () == list.End ()) {
	//	printf("No %s found.\n", info_str);
	//	return -1;
	//}

	//int numNodes = 0;
	//for (xn::NodeInfoList::Iterator nodeIt = list.Begin (); nodeIt != list.End (); ++nodeIt, numNodes++) {
	//	const xn::NodeInfo& info = *nodeIt;
	//	const XnProductionNodeDescription& description = info.GetDescription();
	//	printf("device %d vendor %s name %s, instance %s\n", numNodes, description.strVendor, description.strName, info.GetInstanceName());
	//}

	//std::cout << "Finishing enumerating: " << info_str << std::endl;

	//xn::NodeInfoList::Iterator nodeItUser  = list.Begin();
	//xn::NodeInfo info = *nodeItUser;

	//_g_context.CreateProductionTree(info);

	//UserGenerator* g_user0= new UserGenerator();
	//UserGenerator* g_user1= new UserGenerator();

	//status = info.GetInstance(*g_user0);
	//if (!g_user0->IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
	//		!g_user0->IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
	//	{
	//		printf("User generator doesn't support either skeleton or pose detection.\n");
	//		return 0;
	//	}
	//_g_user.push_back(g_user0);
	//
	//XnStatus rc;
	//XnCallbackHandle hUserCBs, hCalibrationStartCB, hCalibrationCompleteCB, hPoseCBs;
	//rc = g_user0->RegisterUserCallbacks(NewUser_0, LostUser_0, NULL, hUserCBs);
	//CHECK_RC(rc,"Register 0\n");
	////Second user generator:

	//nodeItUser++;//nodeItUser++,nodeItUser++,nodeItUser++,nodeItUser++; //select the fourth node
	//info = *nodeItUser;
	//
	//_g_context.CreateProductionTree(info);
	//status = info.GetInstance(*g_user1);
	//if (!g_user1->IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
	//		!g_user1->IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
	//	{
	//		printf("User generator doesn't support either skeleton or pose detection.\n");
	//		return 0;
	//	}

	//_g_user.push_back(g_user1);


	//rc = g_user1->RegisterUserCallbacks(NewUser_1, LostUser_1, NULL, hUserCBs);
	//CHECK_RC(rc,"Register 1\n");
}


int _mk_init_1() {

	XnStatus status;
	
	status = _g_context[1].Init();

	static xn::NodeInfoList node_info_list;
	static xn::NodeInfoList depth_nodes;
	static xn::NodeInfoList image_nodes;
	static xn::NodeInfoList user_nodes;

	status = _g_context[1].EnumerateProductionTrees (XN_NODE_TYPE_DEVICE, NULL, node_info_list);

	if (status != XN_STATUS_OK && node_info_list.Begin () != node_info_list.End ()) { 
		printf ("Enumerating devices failed. Reason: %s", xnGetStatusString (status)); 
		return -1;
	}
	else if (node_info_list.Begin () == node_info_list.End ()) {
		printf("No devices found.\n");
		return -1;
	}

	for (xn::NodeInfoList::Iterator nodeIt = node_info_list.Begin (); nodeIt != node_info_list.End (); ++nodeIt) {
		_nKinects++;
		xn::NodeInfo info = *nodeIt;
		const XnProductionNodeDescription& description = info.GetDescription();
		printf("image: vendor (%s) name (%s), \ninstance (%s), \ndescription (%s)\n\n\n",	description.strVendor, 
																				description.strName, 
																				info.GetInstanceName(),
																				info.GetCreationInfo());
	}

	status = _g_context[1].EnumerateProductionTrees (XN_NODE_TYPE_IMAGE, NULL, image_nodes, NULL);

	if (status != XN_STATUS_OK && image_nodes.Begin () != image_nodes.End ()) { 
		printf ("Enumerating devices failed. Reason: %s", xnGetStatusString (status)); 
		return -1;
	}
	else if (image_nodes.Begin () == image_nodes.End ()) {
		printf("No devices found.\n");
		return -1;
	}

	status = _g_context[1].EnumerateProductionTrees (XN_NODE_TYPE_DEPTH, NULL, depth_nodes, NULL);

	if (status != XN_STATUS_OK && depth_nodes.Begin () != depth_nodes.End ()) { 
		printf ("Enumerating devices failed. Reason: %s", xnGetStatusString (status)); 
		return -1;
	}
	else if (depth_nodes.Begin () == depth_nodes.End ()) {
		printf("No devices found.\n");
		return -1;
	}

	status = _g_context[1].EnumerateProductionTrees (XN_NODE_TYPE_USER, NULL, user_nodes, NULL);

	if (status != XN_STATUS_OK && user_nodes.Begin () != user_nodes.End ()) { 
		printf ("Enumerating devices failed. Reason: %s", xnGetStatusString (status)); 
		return -1;
	}
	else if (user_nodes.Begin () == user_nodes.End ()) {
		printf("No devices found.\n");
		return -1;
	}

	int i = 0;

	

	i = 0;
	//Image Generators
	for (xn::NodeInfoList::Iterator nodeIt =image_nodes.Begin(); nodeIt != image_nodes.End(); ++nodeIt, i++) {
		if(i == 1){
		xn::NodeInfo info = *nodeIt;

		XnMapOutputMode mode;
		mode.nXRes	= 640;
		mode.nYRes	= 480;
		mode.nFPS	= 30;

		status = _g_context[1].CreateProductionTree (info);

		ImageGenerator* g_image = new ImageGenerator();
		ImageMetaData* g_imageMD = new ImageMetaData();

		status = info.GetInstance (*g_image);

		status = g_image->SetMapOutputMode(mode);
		g_image->GetMetaData(*g_imageMD);
		g_image->StartGenerating();

		_g_image.push_back(g_image);
		_g_imageMD.push_back(g_imageMD);
		}
	}

	i = 0;
	//Depth Generators
	for (xn::NodeInfoList::Iterator nodeIt =depth_nodes.Begin(); nodeIt != depth_nodes.End(); ++nodeIt, i++) {
		if(i == 1){
		xn::NodeInfo info = *nodeIt;

		XnMapOutputMode mode;
		mode.nXRes	= 640;
		mode.nYRes	= 480;
		mode.nFPS	= 30;

		status = _g_context[1].CreateProductionTree (info);

		DepthGenerator* g_depth = new DepthGenerator();
		DepthMetaData* g_depthMD = new DepthMetaData();

		status = info.GetInstance (*g_depth);

		g_depth->SetMapOutputMode(mode);
		g_depth->GetMetaData(*g_depthMD);
		g_depth->StartGenerating();

		_g_depth.push_back(g_depth);
		_g_depthMD.push_back(g_depthMD);
		}
	}

	i = 0;

	//User Generators
	for (xn::NodeInfoList::Iterator nodeIt =user_nodes.Begin(); nodeIt != user_nodes.End(); ++nodeIt, i++) {
		if(i == 1){
		xn::NodeInfo info = *nodeIt;
		printf("user %d\n",i);

			status = _g_context[1].CreateProductionTree (info);

			UserGenerator* g_user= new UserGenerator();

			status = info.GetInstance (*g_user);
			CHECK_RC(status,"User get instance\n");
		
			if (!g_user->IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
				!g_user->IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
			{
				printf("User generator doesn't support either skeleton or pose detection.\n");
				return 0;
			}
			g_user->GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

			status = g_user->StartGenerating();
			CHECK_RC(status,"User start generating\n");
		
			_g_user.push_back(g_user);
		
			XnStatus rc;
			XnCallbackHandle hUserCBs, hCalibrationStartCB, hCalibrationCompleteCB, hPoseCBs;
			//if(i == 0){
			//	rc = g_user->RegisterUserCallbacks(NewUser_0, LostUser_0, NULL, hUserCBs);
			//	CHECK_RC(rc,"Register 0\n");
			//}
			//else{
				rc = g_user->RegisterUserCallbacks(NewUser_1, LostUser_1, NULL, hUserCBs);
				CHECK_RC(rc,"Register 1\n");
			//}
		}
	}

	//for (int i = 0; i < _nKinects; i++) {
		_g_image[1]->GetMirrorCap().SetMirror(false);
		_g_depth[1]->GetAlternativeViewPointCap().SetViewPoint(*_g_image[1]);
		_g_depth[1]->GetMirrorCap().SetMirror(false);
	//}
 
	status =_g_context[1].StartGeneratingAll();
	
	return 1;

	
	//xn::NodeInfoList list;
	//std::string info_str("user nodes");
	//status = _g_context.EnumerateProductionTrees (XN_NODE_TYPE_USER, NULL, list, NULL);

	//if (status != XN_STATUS_OK && list.Begin () != list.End ()) { 
	//	printf ("Enumerating %s failed. Reason: %s", info_str, xnGetStatusString (status)); 
	//	return -1;
	//}
	//else if (list.Begin () == list.End ()) {
	//	printf("No %s found.\n", info_str);
	//	return -1;
	//}

	//int numNodes = 0;
	//for (xn::NodeInfoList::Iterator nodeIt = list.Begin (); nodeIt != list.End (); ++nodeIt, numNodes++) {
	//	const xn::NodeInfo& info = *nodeIt;
	//	const XnProductionNodeDescription& description = info.GetDescription();
	//	printf("device %d vendor %s name %s, instance %s\n", numNodes, description.strVendor, description.strName, info.GetInstanceName());
	//}

	//std::cout << "Finishing enumerating: " << info_str << std::endl;

	//xn::NodeInfoList::Iterator nodeItUser  = list.Begin();
	//xn::NodeInfo info = *nodeItUser;

	//_g_context.CreateProductionTree(info);

	//UserGenerator* g_user0= new UserGenerator();
	//UserGenerator* g_user1= new UserGenerator();

	//status = info.GetInstance(*g_user0);
	//if (!g_user0->IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
	//		!g_user0->IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
	//	{
	//		printf("User generator doesn't support either skeleton or pose detection.\n");
	//		return 0;
	//	}
	//_g_user.push_back(g_user0);
	//
	//XnStatus rc;
	//XnCallbackHandle hUserCBs, hCalibrationStartCB, hCalibrationCompleteCB, hPoseCBs;
	//rc = g_user0->RegisterUserCallbacks(NewUser_0, LostUser_0, NULL, hUserCBs);
	//CHECK_RC(rc,"Register 0\n");
	////Second user generator:

	//nodeItUser++;//nodeItUser++,nodeItUser++,nodeItUser++,nodeItUser++; //select the fourth node
	//info = *nodeItUser;
	//
	//_g_context.CreateProductionTree(info);
	//status = info.GetInstance(*g_user1);
	//if (!g_user1->IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
	//		!g_user1->IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
	//	{
	//		printf("User generator doesn't support either skeleton or pose detection.\n");
	//		return 0;
	//	}

	//_g_user.push_back(g_user1);


	//rc = g_user1->RegisterUserCallbacks(NewUser_1, LostUser_1, NULL, hUserCBs);
	//CHECK_RC(rc,"Register 1\n");
}
*/