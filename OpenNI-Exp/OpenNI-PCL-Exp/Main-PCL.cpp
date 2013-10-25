#include "_ID.h"

#include "pcl_visualizer.h"
#include "pcl_recording_visualizer.h"
#include "pcl_foot.h"
#include "pcl_topview.h"
#include "Walkys-PCL.h"
#include "pcl_nikinect.h"
#include "pcl_thread_viewer.h"
#include "pcl_nithreadedkinect.h"
#include "pcl_mirror.h"
#include "pcl_multi_kinect.h"

int main(int argc, char* argv[]){
	int result = -1;

	//result = main_pcl_visualizer(argc,argv);
	//result = main_pcl_visualizer_simple(argc,argv);
	//result = main_pcl_recording_visualizer(argc,argv);
	//result = main_pcl_foot(argc,argv);
	//result = main_pcl_topview(argc,argv);
	//result = main_walkys(argc,argv);
	//result = main_pcl_ni_kinect(argc,argv);
	//result = main_thread_viewer(argc,argv);
	//result = main_nithreadedkinect(argc,argv);
	//result = main_pcl_mirrors(argc,argv);
	//result = main_pcl_multi_kinect(argc,argv);

#ifdef _CCG
	result = main_pcl_multi_kinect(argc,argv);
#endif

#ifdef _HOME
	result = main_thread_viewer(argc,argv);
#endif

	return result;
}