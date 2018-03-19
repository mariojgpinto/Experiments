#include "Kinect2.h"
#include "Sample_Body.h"
#include "Sample_Color.h"
#include "Sample_Depth.h"
#include "Sample_Background.h"

int main(int argc, char* argv[]){
	int result = 0; 

	//result = main_kinect_2(argc,argv);
	//result = main_sample_body(argc, argv);
	//result = main_sample_color(argc, argv);
	//result = main_sample_depth(argc, argv);
	//result = main_sample_body(argc, argv);
	result = main_sample_background(argc, argv);




	return result;
}
