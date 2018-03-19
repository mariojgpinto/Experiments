#include <_ID.h>
#include <stdio.h>
#include <cmath>

#include "SimpleViewer.h"
#include "MultiViewer.h"
#include "SimpleUserViewer.h"
#include "MultiUserViewer.h"
#include "MultiNIKinect2.h"
#include "FaceTracker.h"
#include "MultiTrackingColor.h"
#include "FaceTemplateMatch.h"
#include "MultiTrackingFeatures.h"
#include "MultiTrackingBio.h"
#include "MultiNIKinect2ColorAdjust.h"
#include "CameraCalibration.h"

inline int round(float x) { return (floor(x + 0.5)); }

int main(int argc, char* argv[]){
	int result = -1;

	//result = main_simple_viewer(argc, argv);
	//result = main_multi_viewer(argc, argv);
	//result = main_simple_user_viewer(argc, argv);
	//result = main_multi_user_viewer(argc, argv);
	//result = main_multi_nikinect2(argc, argv);
	//result = main_face_tracker(argc, argv);
	//result = main_face_template_match(argc, argv);
	//result = main_multi_tracking_color(argc, argv);
	//result = main_multi_tracking_features(argc, argv);
	//result = main_multi_tracking_bio(argc, argv);
	//result = main_multi_nikinect2_color_adjust(argc, argv);
	//result = main_kinect_calibration(argc, argv);
	
#ifdef _CCG
	//float r1 = sqrt((double)1);
	//float r2 = sqrt((double)2);
	//float r3 = sqrt((double)3);
	//float r4 = sqrt((double)4);
	//float r5 = sqrt((double)5);

	//int i0 = round (sqrt((double)3));
	//int i1 = round (r1);
	//int i2 = round (r2);
	//int i3 = round (r3);
	//int i4 = round (r4);
	//int i5 = round (r5);

	result = main_simple_viewer(argc, argv);
#endif

#ifdef _HOME
	result = main_simple_viewer(argc,argv);
#endif

	printf("Result: %d\n",result);
	//getchar();

	return result;
}