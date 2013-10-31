#include <_ID.h>
#include <stdio.h>

#include "SimpleViewer.h"
#include "MultiViewer.h"
#include "SimpleUserViewer.h"
#include "MultiUserViewer.h"
#include "MultiNIKinect2.h"
#include "FaceTracker.h"


int main(int argc, char* argv[]){
	int result = -1;

	//result = main_simple_viewer(argc, argv);
	//result = main_multi_viewer(argc, argv);
	//result = main_simple_user_viewer(argc, argv);
	//result = main_multi_user_viewer(argc, argv);
	//result = main_multi_nikinect2(argc, argv);
	//result = main_face_tracker(argc, argv);

#ifdef _CCG
	result = main_face_tracker(argc, argv);
#endif

#ifdef _HOME
	result = main_simple_viewer(argc,argv);
#endif

	printf("Result: %d\n",result);
	//getchar();

	return result;
}