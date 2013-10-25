#include <_ID.h>
#include <stdio.h>

#include "SimpleViewer3D.h"
#include "MultiViewer3D.h"
#include "User3D.h"
#include "TopView2.h"

int main(int argc, char* argv[]){
	int result = -1;

	//result = main_simple_viewer_3d(argc, argv);
	//result = main_user_3d(argc, argv);
	//result = main_multi_viewer_3d(argc, argv);
	//result = main_top_view_2(argc, argv);

#ifdef _CCG
	result = main_top_view_2(argc, argv);
#endif

#ifdef _HOME
	//result = main_simple_viewer_3d(argc,argv);
#endif

	printf("Result: %d\n",result);
	//getchar();

	return result;
}