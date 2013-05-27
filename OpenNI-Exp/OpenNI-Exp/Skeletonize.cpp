#include "Slice.h"

#include <XnOS.h>
#include <math.h>

#include <XnCppWrapper.h>
using namespace xn;

#include <opencv2\opencv.hpp>

#define SAMPLE_XML_PATH "C:\\Dev\\External\\OpenNI\\Data\\SamplesConfig.xml"

void compute_color_encoded_depth(const cv::Mat1f& depth_im, cv::Mat3b& color_depth_im,
                                     double* i_min_val, double* i_max_val);
float distanceToPlane(const XnPoint3D& p, float a, float b, float c, float d);


#define NORTH 1
#define SOUTH 3


// 1-neighbors of pixel.
int nays8(IplImage *im, int r, int c) {
	CvScalar pixel;
	int blue, k = 0, i, j;

  for (i = r-1; i <= r+1; i++) 
		for (j = c-1; j <= c+1; j++) 
			if (i != r || c != j) {
				pixel = cvGet2D(im, i, j);
				blue = pixel.val[0];
				if (blue >= 1)
					k++;
			}

	return k;
}

int connectivity(IplImage *im, int r, int c) {
	int N = 0, b1, b2;
	CvScalar pixel;

	pixel = cvGet2D(im, r, c+1);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r-1, c+1);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0) 
		N++;

	pixel = cvGet2D(im, r-1, c+1);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r-1, c);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0)
		N++;

	pixel = cvGet2D(im, r-1, c);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r-1, c-1);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0)
		N++;

	pixel = cvGet2D(im, r-1, c-1);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r, c-1);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0)
		N++;

	pixel = cvGet2D(im, r, c-1);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r+1, c-1);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0)
		N++;

	pixel = cvGet2D(im, r+1, c-1);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r+1, c);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0)
		N++;

	pixel = cvGet2D(im, r+1, c);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r+1, c+1);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0)
		N++;

	pixel = cvGet2D(im, r+1, c+1);
	b1 = pixel.val[0];
	pixel = cvGet2D(im, r, c+1);
	b2 = pixel.val[0];
	if (b1 >= 1 && b2 == 0)
		N++;

	return N;
}

void deleteCB(IplImage *im, IplImage *tmp) {
	int w, h, blue, i, j;
	CvScalar pixel;

	w = im->width;
	h = im->height;

	for (i = 1; i < h-1; i++)
		for (int j = 1; j < w-1; j++) {
			pixel = cvGet2D(tmp, i, j);
			blue = pixel.val[0];
			if (blue == 1) {
				pixel.val[0] = 0;
				cvSet2D(im, i, j, pixel);
				cvSet2D(tmp, i, j, pixel);
			}
		}
}

void stair(IplImage *im, IplImage *tmp, int dir) {
	int i, j, b1, b2, b3, b4, b5, b6, b7, b8, b9, w, h;
	CvScalar pixel;
	int N, S, E, W, NE, NW, SE, SW, C;

	w = im->width;
	h = im->height;

	if (dir == NORTH)
		for (i = 1; i < h-1; i++)
			for (j = 1; j < w-1; j++) {
				pixel = cvGet2D(im, i-1, j-1);
				b1 = pixel.val[0];
				pixel = cvGet2D(im, i-1, j);
				b2 = pixel.val[0];
				pixel = cvGet2D(im, i-1, j+1);
				b3 = pixel.val[0];
				pixel = cvGet2D(im, i, j-1);
				b4 = pixel.val[0];
				pixel = cvGet2D(im, i, j);
				b5 = pixel.val[0];
				pixel = cvGet2D(im, i, j+1);
				b6 = pixel.val[0];
				pixel = cvGet2D(im, i+1, j-1);
				b7 = pixel.val[0];
				pixel = cvGet2D(im, i+1, j);
				b8 = pixel.val[0];
				pixel = cvGet2D(im, i+1, j+1);
				b9 = pixel.val[0];
				if (b1 == 1)
					NW = 1;
				else
					NW = 0;
				if (b2 == 1)
					N = 1;
				else
					N = 0;
				if (b3 == 1)
					NE = 1;
				else
					NE = 0;
				if (b4 == 1)
					W = 1;
				else
					W = 0;
				if (b5 == 1)
					C = 1;
				else
					C = 0;
				if (b6 == 1)
					E = 1;
				else
					E = 0;
				if (b7 == 1)
					SW = 1;
				else
					SW = 0;
				if (b8 == 1)
					S = 1;
				else
					S = 0;
				if (b9 == 1)
					SE = 1;
				else
					SE = 0;

				if (dir == NORTH) {
					if (C && !(N && ((E && !NE && !SW && (!W || !S)) || 
						 (W && !NW && !SE && (!E || !S))))) {
						pixel.val[0] = 0;
						cvSet2D(tmp, i, j, pixel);
					} else {
						pixel.val[0] = 1;
						cvSet2D(tmp, i, j, pixel);
					}
				} else if (dir == SOUTH) {
					if (C && !(S && ((E && !SE && !NW && (!W || !N)) || 
						 (W && !SW && !NE && (!E || !N))))) {
						pixel.val[0] = 0;
						cvSet2D(tmp, i, j, pixel);
					} else {
						pixel.val[0] = 1;
						cvSet2D(tmp, i, j, pixel);
					}
				}
			}
}

// Zhang-Suen algorithm.
void skeletonize(IplImage *im) {
	int janelaAH[][2] = {
		{1, 0}, {0, -1}, {-1, 0}, {0, 1}
	};
	int janelaH[][2] = {
		{0, -1}, {1, 0}, {0, 1}, {-1, 0}
	};
	int aBlue[6];
	int w, h, i, v, j, k, blue, lin, col, iJanela, again = 1;
	CvScalar pixel, pixOut;	
	IplImage *tmp = 0;
	
	w = im->width;
	h = im->height;
	tmp = cvCreateImage(cvGetSize(im), IPL_DEPTH_8U, 1);
	
  for (i = 0; i < h; i++) {
		for (j = 0; j < w; j++) { 
			pixel = cvGet2D(im, i, j);
			blue = pixel.val[0];
			if (blue > 0)
				pixel.val[0] = 0;
			else
				pixel.val[0] = 1;
			cvSet2D(im, i, j, pixel);
			pixOut.val[0] = 0;
			cvSet2D(tmp, i, j, pixOut);
		}
	}

	while (again) {
		again = 0;
  	for (i = 1; i < h-1; i++) 
			for (j = 1; j < w-1; j++) { 
				pixel = cvGet2D(im, i, j);
				blue = pixel.val[0];
				if (blue != 1)
					continue;
				k = nays8(im, i, j);
				iJanela = 0;
				if ((k >= 2 && k <= 6) && connectivity(im, i, j) == 1) {
					for (v = 0; v < 6; v++) {
						col = j + janelaAH[iJanela][0];
						lin = i + janelaAH[iJanela][1];
						pixel = cvGet2D(im, lin, col);
						aBlue[v] = pixel.val[0];
						iJanela++;
						if (v == 2) 
							iJanela = 1;
					}
					if (aBlue[0]*aBlue[1]*aBlue[2] == 0 &&
							aBlue[3]*aBlue[4]*aBlue[5] == 0) {
						pixOut.val[0] = 1;
						cvSet2D(tmp, i, j, pixOut);
						again = 1;
					}
				}		// if ((k >= 2...
			}		// for (j = 1;...

			deleteCB(im, tmp);
			if (!again)
				break;

  	for (i = 1; i < h-1; i++) 
			for (j = 1; j < w-1; j++) { 
				pixel = cvGet2D(im, i, j);
				blue = pixel.val[0];
				if (blue != 1)
					continue;
				k = nays8(im, i, j);
				iJanela = 0;
				if ((k >= 2 && k <= 6) && connectivity(im, i, j) == 1) {
					for (v = 0; v < 6; v++) {
						col = j + janelaH[iJanela][0];
						lin = i + janelaH[iJanela][1];
						pixel = cvGet2D(im, lin, col);
						aBlue[v] = pixel.val[0];
						iJanela++;
						if (v == 2) 
							iJanela = 1;
					}
					if (aBlue[0]*aBlue[1]*aBlue[2] == 0 &&
							aBlue[3]*aBlue[4]*aBlue[5] == 0) {
						pixOut.val[0] = 1;
						cvSet2D(tmp, i, j, pixOut);
						again = 1;
					}
				}		// if ((k >= 2...
			}		// for (j = 1;...

		deleteCB(im, tmp);
	}		// while

	stair(im, tmp, NORTH);
	deleteCB(im, tmp);
	stair(im, tmp, SOUTH);
	deleteCB(im, tmp);

  for (i = 1; i < h-1; i++) 
		for (j = 1; j < w-1; j++) { 
			pixel = cvGet2D(im, i, j);
			blue = pixel.val[0];
			if (blue > 0)
				pixel.val[0] = 0;
			else
				pixel.val[0] = 255;
			cvSet2D(im, i, j, pixel);
		}
}		// End skeletonize


int main_skeletonize(int argc, char* argv[]){
	Context _context;
	ScriptNode _scriptNode;
	DepthGenerator _depth;
	ImageGenerator _image;
	DepthMetaData _depthMD;
	ImageMetaData _imageMD;
	SceneMetaData _sceneMD;

	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;


	xn::SceneAnalyzer xn_scene;
	bool remove_floor = false;
	XnPlane3D floorCoords;
    XnPoint3D floorPoint;
	XnPoint3D * pointList = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 
	XnPoint3D * realWorld = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 
	float a = 0,b = 0,c = 0,d = 0;
	float a2 = 0,b2 = 0,c2 = 0,d2 = 0;

	XnStatus rc;

	int _min_bar = 400;
	int _max_bar = 1200;
	int _thresh = 175;
	int _thresh_floor = 185;
	int _floor_range = 10;
	int _kernel = 3;
	cv::namedWindow("Ranged Image");
	cv::createTrackbar("MinDepth", "Ranged Image", &_min_bar, 5000, NULL);
	cv::createTrackbar("MaxDepth", "Ranged Image", &_max_bar, 5000, NULL);
	cv::createTrackbar("FloorHeight", "Ranged Image", &_thresh, 400, NULL);
	cv::createTrackbar("FloorThresh", "Ranged Image", &_floor_range, 200, NULL);
	cv::createTrackbar("Kernel", "Ranged Image", &_kernel, 11, NULL);

	{
	EnumerationErrors errors;
	//rc = _context.InitFromXmlFile(SAMPLE_XML_PATH, _scriptNode, &errors);
	rc = _context.Init();
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

	rc = _context.OpenFileRecording("C:\\Dev\\Walkys\\Project\\Data\\foot_1_left.oni");
	//printf("Open failed: %s\n", xnGetStatusString(rc));
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


	rc = xn_scene.Create(_context);
//rc = _context.FindExistingNode(XN_NODE_TYPE_SCENE, xn_scene);
	if (rc != XN_STATUS_OK)
	{
		printf("No Scene Analyzer\n.");
		getchar();
		return 1;
	}

	xn::UserGenerator _user;
	rc = _user.Create(_context);
	//rc = _context.FindExistingNode(XN_NODE_TYPE_SCENE, xn_scene);
    if (rc != XN_STATUS_OK)
    {
		printf("No _User Analyzer\n.");
		getchar();
		return 1;
    }
	
	//xn_scene.GetMetaData(_sceneMD);

	_context.StartGeneratingAll();
	}
	cv::RNG rng(12345);
	char ch = 0;



	cv::namedWindow("Skel");
	int thresh = 33;
	cv::createTrackbar("thresh","Skel",&thresh,255,NULL);

	while((ch = cv::waitKey(23)) != 27){
		XnStatus rc = XN_STATUS_OK;

		// Read a new frame
		rc = _context.WaitAnyUpdateAll();
		if (rc != XN_STATUS_OK)
		{
			printf("Read failed: %s\n", xnGetStatusString(rc));
			break;
		}

		_depth.GetMetaData(_depthMD);
		_image.GetMetaData(_imageMD);
		//xn_scene.GetMetaData(_sceneMD);



		cv::Mat depthMat16UC1(480, 640,CV_16UC1, (void*) _depthMD.Data());
		cv::Mat depthMat8UC1;
		depthMat16UC1.convertTo(depthMat8UC1, CV_8UC1,0.05);
		//cv::Mat depthMat8UC1;
		//depthMat16UC1.convertTo(depthMat8UC1, CV_8UC1,0.05);

		cv::Mat color(480,640,CV_8UC3,(void*) _imageMD.Data());
		cv::Mat color2;
		cv::cvtColor(color,color2,CV_RGB2BGR);

		//cv::Mat mask; cv::threshold(depthMat8UC1,mask,1,255,CV_THRESH_BINARY);
		cv::Mat mask_cv;			
		cv::inRange(depthMat16UC1,_min_bar,_max_bar,mask_cv);
		cv::Mat color3;
		cv::Mat color4;

		double mm, nn;
		cv::Mat asd;
		depthMat16UC1.copyTo(asd,mask_cv);
		cv::minMaxLoc(asd,&mm,&nn);

		//int XN_VGA_Y_RES = 480;
		//int XN_VGA_X_RES = 640;
		if(remove_floor){
			XnPoint3D point2;

			uchar* ptr = mask_cv.data;
			for(int y=0; y<XN_VGA_Y_RES; y++) { 
				for(int x=0; x<XN_VGA_X_RES; x++) { 
					XnPoint3D point1;
					point1.X = x; 
					point1.Y = y; 
					point1.Z = _depthMD[y * XN_VGA_X_RES + x]; 

					pointList[y * XN_VGA_X_RES + x] = point1;
				}
			} 

			_depth.ConvertProjectiveToRealWorld(XN_VGA_Y_RES*XN_VGA_X_RES, pointList, realWorld); 

			_thresh_floor = _thresh + _floor_range;

			floorPoint = floorCoords.ptPoint;
			floorPoint.X += floorCoords.vNormal.X * _thresh_floor;
			floorPoint.Y += floorCoords.vNormal.Y * _thresh_floor;
			floorPoint.Z += floorCoords.vNormal.Z * _thresh_floor;
			//floorPoint.Z -= _thresh_floor;
			a2 = floorCoords.vNormal.X;
			b2 = floorCoords.vNormal.Y;
			c2 = floorCoords.vNormal.Z;
			d2 = -(a2*floorPoint.X + b2*floorPoint.Y + c2*floorPoint.Z);

			uchar* ptr2 = depthMat16UC1.data;
			for(int y=0; y<XN_VGA_Y_RES; y++) { 
				for(int x=0; x<XN_VGA_X_RES; x++) { 
					if(realWorld[y * XN_VGA_X_RES + x].Z > 0.0 && ptr[y * XN_VGA_X_RES + x]){
						float value1 = distanceToPlane(realWorld[y * XN_VGA_X_RES + x],a,b,c,d) ;
						float value2 = distanceToPlane(realWorld[y * XN_VGA_X_RES + x],a2,b2,c2,d2) ;
						if( value1 < _thresh &&  value2 < _thresh){
							ptr[y * XN_VGA_X_RES + x]=255;
							//double as = depthMat16UC1.ptr<float>(x)[y];
							//printf("");
							//ptr2[y * XN_VGA_X_RES + x] = (uchar)abs(value);
							//((uchar*)(mask.data + mask.step*y))[x]=0;
							//pointList[y * XN_VGA_X_RES + x].Y = y; 
						}
						else{
							ptr[y * XN_VGA_X_RES + x]=0;
						}
						//if( value > 300){
						//	printf("big");
						//}
					}
					else{
						printf("");
					}
				} 
			} 
			cv::Mat distT_mask;
			cv::Mat distT;
			cv::Mat distT_8UC1;
			cv::Mat distT_Laplacian;
			cv::Mat distT_Laplacian_abs;
			cv::Mat distT_Laplacian_abs_thresh;
			cv::Mat distT_Laplacian_abs_thresh_erode;
			cv::Mat distT_Laplacian_abs_thresh_dilate;

			

			cv::erode(mask_cv,distT_mask,cv::Mat(5,5,CV_8UC1));			
			cv::dilate(distT_mask,distT_mask,cv::Mat(11,11,CV_8UC1));
			cv::erode(distT_mask,distT_mask,cv::Mat(5,5,CV_8UC1));
			//cv::imshow("mask2",distT_mask);

			cv::distanceTransform(distT_mask,distT,CV_DIST_L2,5);
			
			distT.convertTo(distT_8UC1,CV_8UC1);

			cv::Laplacian(distT_8UC1,distT_Laplacian,CV_32F,5,1,0,4);

			convertScaleAbs( distT_Laplacian, distT_Laplacian_abs );

			float v = ((float)thresh);
			cv::threshold(distT_Laplacian_abs,distT_Laplacian_abs_thresh,v,255,CV_THRESH_BINARY);
			

			cv::dilate(distT_Laplacian_abs_thresh,distT_Laplacian_abs_thresh_dilate,cv::Mat(9,9,CV_8UC1));
			cv::erode(distT_Laplacian_abs_thresh_dilate,distT_Laplacian_abs_thresh_erode,cv::Mat(7,7,CV_8UC1));

			cv::Mat top_view_color = cv::Mat::zeros(distT_Laplacian_abs_thresh.size(), CV_8UC1);

			cv::vector<cv::vector<cv::Point> > contours;
			cv::vector<cv::Vec4i> hierarchy;
			
			cv::findContours( distT_Laplacian_abs_thresh_erode, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

			for(unsigned int i = 0; i< contours.size(); i++ ){
				if(contours[i].size() > 75){
					//cv::Scalar clr = cv::Scalar(255);
					//for(unsigned int j = 0 ; j < contours[i].size() ; j++){
					//	cv::circle(top_view_color,contours[i][j],1,clr,-1);
					//}

					cv::Scalar clr = cv::Scalar(255);// rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
					cv::Rect rect = cv::boundingRect(contours[i]);
					drawContours( top_view_color, contours, i, clr, -1, 1, hierarchy, 0, cv::Point() );
					////cv::rectangle(top_view_color,rect, clr);
				}
			}
			
			
			cv::imshow("Skel",top_view_color + depthMat8UC1);
			

			//cv::Mat mask_temp;
			//cv::Mat mask_inv;
			//cv::erode(mask_cv,mask_temp,cv::Mat(5,5,CV_8UC1));
			//cv::dilate(mask_cv,mask_temp,cv::Mat(5,5,CV_8UC1));

			//cv::threshold(mask_temp,mask_inv,1,255,CV_THRESH_BINARY_INV);
			//cv::imshow("Mask",mask_inv);
			//IplImage ipl_image = mask_inv.operator IplImage();
			//skeletonize(&ipl_image);
			//cvShowImage("IPL",&ipl_image);
			

			

		}
		
		cv::Mat3b cor(480,640);
		double min = _min_bar,max = _max_bar;
		compute_color_encoded_depth(depthMat16UC1,cor,&min,&max);

		cor.copyTo(color3,mask_cv);
		//color2.copyTo(color3,mask_cv);

		//cv::vector<cv::vector<cv::Point> > contours;
		//cv::vector<cv::Vec4i> hierarchy;

		//cv::findContours( mask_cv, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

		//for(unsigned int i = 0; i< contours.size(); i++ ){
		//	if(contours[i].size() > 100){
		//		cv::Scalar clr = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		//		cv::Rect rect = cv::boundingRect(contours[i]);

		//		cv::Moments mm = moments( contours[i], false );
		//		cv::Point2f center = cv::Point2f((float)(mm.m10/mm.m00) , (float)(mm.m01/mm.m00));

		//		drawContours( color3, contours, i, clr, 2, 8, hierarchy, 0, cv::Point() );

		//		cv::circle(color3,center,3,clr);
		//	}
		//}

		//cv::circle(color3, cv::Point(320,350),5,cv::Scalar(255,0,0));

		//cv::imshow("depth",depthMat8UC1);
		cv::imshow("Ranged Image",color3);
		//cv::imshow("color",);

		++_frame_counter;
		if (_frame_counter == 15)
		{
			double current_tick = cv::getTickCount();
			_frame_rate = _frame_counter / ((current_tick - _last_tick)/cv::getTickFrequency());
			_last_tick = current_tick;
			_frame_counter = 0;
			printf("%.2f\n",_frame_rate);
		}

		if(ch == ' '){
			cv::Mat temp; depthMat16UC1.copyTo(temp,mask_cv);
			cv::Mat depthMat8UC1;
			temp.convertTo(depthMat8UC1, CV_8UC1,0.05);

			static int n = 1;
			char buff[128];
			sprintf(buff,"Images\\img_%02d.png",n++);
			cv::imwrite(buff,depthMat8UC1);
			cv::imshow("depth",depthMat8UC1);
		}

		if(ch == 'f'){
			if(remove_floor){
				remove_floor = false;
			}
			else{
				rc = xn_scene.GetFloor( floorCoords);
				//rc = xnGetFloor(xn_scene, &floorCoords);
				if(rc == XN_STATUS_OK){
					std::cout << floorCoords.vNormal.X << " " << floorCoords.vNormal.Y << " " << floorCoords.vNormal.Z << std::endl;
					std::cout << floorCoords.ptPoint.X << " " << floorCoords.ptPoint.Y << " " << floorCoords.ptPoint.Z  << "\n" << std::endl;

					a = floorCoords.vNormal.X;
					b = floorCoords.vNormal.Y;
					c = floorCoords.vNormal.Z;

					XnPoint3D pt1; pt1.X = 320; pt1.Y = 350; pt1.Z = _depthMD[ 350*640+ 320 ];
					XnPoint3D pt2;
					XnPoint3D pt3;
					//_depth.ConvertProjectiveToRealWorld(1,&floorCoords.ptPoint,&pt2);
					//_depth.ConvertProjectiveToRealWorld(1,&pt1,&pt2);
					_depth.ConvertRealWorldToProjective(1,&floorCoords.ptPoint,&pt3);

					d = -(a*floorCoords.ptPoint.X + b*floorCoords.ptPoint.Y + c*floorCoords.ptPoint.Z);

					floorPoint = floorCoords.ptPoint;
					floorPoint.Z-=_thresh_floor;

					a2 = floorCoords.vNormal.X;
					b2 = floorCoords.vNormal.Y;
					c2 = floorCoords.vNormal.Z;

					//_depth.ConvertRealWorldToProjective(1,&floorPoint,&pt2);

					d2 = -(a2*floorPoint.X + b2*floorPoint.Y + c2*floorPoint.Z);

					remove_floor = true;
				}
				else
					printf("Read failed: %s\n", xnGetStatusString(rc));
			}
		}
	}

	return 0;
}