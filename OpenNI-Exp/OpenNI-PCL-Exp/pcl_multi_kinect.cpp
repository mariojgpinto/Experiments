#include "pcl_multi_kinect.h"

#include <_ID.h>

#include "pcl_visualizer.h"

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

int								_nKinects;


#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
	}	


int _mk_init() {

	XnStatus status;
	
	status = _g_context.Init();

	static xn::NodeInfoList node_info_list;
	static xn::NodeInfoList depth_nodes;
	static xn::NodeInfoList image_nodes;

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

	int i = 0;
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

	for (int i = 0; i < _nKinects; i++) {
		_g_image[i]->GetMirrorCap().SetMirror(false);
		_g_depth[i]->GetAlternativeViewPointCap().SetViewPoint(*_g_image[i]);
		_g_depth[i]->GetMirrorCap().SetMirror(false);
	}
 
	status =_g_context.StartGeneratingAll();
	
	return 1;
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

int main_pcl_multi_kinect(int argc, char* argv[]){
	_mk_init();

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
		
		for (int i = 0; i<_nKinects; i++) {
			cv::Mat depthMat16UC1(480, 640,CV_16UC1, (void*) _g_depthMD[i]->Data());
			cv::Mat depthMat8UC1;
			depthMat16UC1.convertTo(depthMat8UC1, CV_8UC1,0.05);

			cv::Mat color(480,640,CV_8UC3,(void*) _g_imageMD[i]->Data());
			cv::Mat color_rgb;
			cv::cvtColor(color,color_rgb,CV_RGB2BGR);
			cv::Mat final;
			color_rgb.copyTo(final,depthMat8UC1);
		
			char buff[100];
			sprintf_s(buff,"Image (%d)",i);
			cv::imshow(buff,final);
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

	return 0;
}