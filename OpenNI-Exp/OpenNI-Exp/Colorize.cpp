#include "Colorize.h"
#include "RemoveFloor.h"

#include <XnOS.h>
#include <math.h>

#include <XnCppWrapper.h>
using namespace xn;

#include <opencv2\opencv.hpp>

#define SAMPLE_XML_PATH "C:\\Dev\\External\\OpenNI\\Data\\SamplesConfig.xml"

void compute_color_encoded_depth(const cv::Mat1f& depth_im, cv::Mat3b& color_depth_im,
                                     double* i_min_val, double* i_max_val)
{
	double min_val, max_val;
	if (i_min_val && i_max_val)
	{
		min_val = *i_min_val;
		max_val = *i_max_val;
	}
	else
	{
		minMaxLoc(depth_im, &min_val, &max_val);
	}

	color_depth_im.create(depth_im.size());
	for (int r = 0; r < depth_im.rows; ++r)
	{
		const float* depth_data = depth_im.ptr<float>(r);
		cv::Vec3b* depth_color_data = color_depth_im.ptr<cv::Vec3b>(r);
		for (int c = 0; c < depth_im.cols; ++c)
		{
			int v = 255*6*(depth_data[c]-min_val)/(max_val-min_val);
			if (v < 0) v = 0;
			char r,g,b;
			int lb = v & 0xff;
			switch (v / 256) {
			case 0:
				r = 255;
				g = 255-lb;
				b = 255-lb;
				break;
			case 1:
				r = 255;
				g = lb;
				b = 0;
				break;
			case 2:
				r = 255-lb;
				g = 255;
				b = 0;
				break;
			case 3:
				r = 0;
				g = 255;
				b = lb;
				break;
			case 4:
				r = 0;
				g = 255-lb;
				b = 255;
				break;
			case 5:
				r = 0;
				g = 0;
				b = 255-lb;
				break;
			default:
				r = 0;
				g = 0;
				b = 0;
				break;
			}
			if (v == 0)
			{
				r = g = b = 0;
			}
			depth_color_data[c] = cv::Vec3b(b,g,r);
		}
	}
}

int main_colorize(int argc, char* argv[]){
	Context _context;
	ScriptNode _scriptNode;
	DepthGenerator _depth;
	ImageGenerator _image;

	DepthMetaData _depthMD;
	ImageMetaData _imageMD;

	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

	XnStatus rc;

	int _min_bar = 400;
	int _max_bar = 1500;

	cv::namedWindow("Ranged Image");
	cv::createTrackbar("MinDepth", "Ranged Image", &_min_bar, 5000, NULL);
	cv::createTrackbar("MaxDepth", "Ranged Image", &_max_bar, 5000, NULL);

	cv::Mat color_rgb;
	
	{
	EnumerationErrors errors;
	rc = _context.InitFromXmlFile(SAMPLE_XML_PATH, _scriptNode, &errors);
	//rc = _context.Init();
	if (rc == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return (rc);
	}
	else if (rc != XN_STATUS_OK)
	{
		printf("Open failed: %s\n", xnGetStatusString(rc));
		return (rc);
	}

	rc = _context.FindExistingNode(XN_NODE_TYPE_DEPTH, _depth);
	if (rc != XN_STATUS_OK)
	{
		rc = _depth.Create(_context);

		if (rc != XN_STATUS_OK)
		{
			printf("No depth node exists! Check your XML.");
			return 1;
		}
	}

	rc = _context.FindExistingNode(XN_NODE_TYPE_IMAGE, _image);
	if (rc != XN_STATUS_OK)
	{
		rc = _image.Create(_context);

		if (rc != XN_STATUS_OK)
		{
			printf("No image node exists! Check your XML.");
			return 1;
		}
	}

	_depth.GetMetaData(_depthMD);
	_image.GetMetaData(_imageMD);

	// Hybrid mode isn't supported in this sample
	if (_imageMD.FullXRes() != _depthMD.FullXRes() || _imageMD.FullYRes() != _depthMD.FullYRes())
	{
		printf ("The device depth and image resolution must be equal!\n");
		return 1;
	}

	// RGB is the only image format supported.
	if (_imageMD.PixelFormat() != XN_PIXEL_FORMAT_RGB24)
	{
		printf("The device image format must be RGB24\n");
		return 1;
	}

	//Sync Images
	_depth.GetAlternativeViewPointCap().SetViewPoint(_image);

	_context.StartGeneratingAll();
	}

	char c = 0;
	while((c = cv::waitKey(23)) != 27){
		// Read a new frame
		rc = _context.WaitAnyUpdateAll();
		if (rc != XN_STATUS_OK)
		{
			printf("Read failed: %s\n", xnGetStatusString(rc));
			break;
		}

		_depth.GetMetaData(_depthMD);
		_image.GetMetaData(_imageMD);

		cv::Mat depthMat16UC1(480, 640,CV_16UC1, (void*) _depthMD.Data());
		cv::Mat color(480,640,CV_8UC3,(void*) _imageMD.Data());
		cv::cvtColor(color,color_rgb,CV_RGB2BGR);
		
		cv::Mat3b colorize;
		double _min = _min_bar;
		double _max = _max_bar;
		compute_color_encoded_depth(depthMat16UC1,colorize,&_min,&_max);
		
		cv::Mat mask_cv;			
		cv::inRange(depthMat16UC1,_min_bar,_max_bar,mask_cv);

		cv::Mat color_range;
		color_rgb.copyTo(color_range,mask_cv);

		cv::imshow("Ranged Image",colorize);
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
}