#include "pcl_visualizer.h"
#include "pcl_recording_visualizer.h"

int main(int argc, char* argv[]){
	int result = -1;

	//result = main_pcl_visualizer(argc,argv);
	result = main_pcl_visualizer_simple(argc,argv);
	//result = main_pcl_recording_visualizer(argc,argv);

	return result;
}