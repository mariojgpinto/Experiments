#include "_ID.h"

#include "NISimpleViewer.h"
#include "NIOpenCV.h"
#include "RemoveFloor.h"
#include "DepthThreshold.h"
#include "Colorize.h"
#include "NIKinectTest.h"
#include "Slice.h"
#include "TopView.h"
#include "Skeletonize.h"
#include "ReadFromFile.h"
#include "TopView_Movement.h"
#include "RDCC.h"
#include "Walkys.h"
#include "multiple_kinect.h"
#include "multi_NIKinect.h"
#include "NIKinectThreaded.h"
#include "KinectRecordingMarie.h"

int main(int argc, char* argv[])
{
	int result = -1;
	//main_simple_viewer(argc,argv);
	//main_opencv(argc,argv);
	//main_remove_floor(argc,argv);
	//main_depth_threshold(argc,argv);
	//main_colorize(argc,argv);
	//main_nikinect_test(argc,argv);
	//main_slice(argc,argv);
	//main_top_view(argc,argv);
	//main_skeletonize(argc,argv);
	//main_read_from_file(argc,argv);
	//result = main_top_view_movement(argc,argv);
	//result = main_rdcc(argc,argv);
	//result = main_walkys(argc,argv);
	//result = main_multiple_kinect(argc, argv);
	//result = main_multi_ni_kinect(argc, argv);
	//result = main_nikinect_threaded(argc,argv);
	//result = main_multi_kinect_usergen(argc,argv);

#ifdef _CCG
	result = 	main_multiple_kinect(argc, argv);
#endif

#ifdef _HOME
	result = main_walkys(argc,argv);
#endif

	return result;
}

