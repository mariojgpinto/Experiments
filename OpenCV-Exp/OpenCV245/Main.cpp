#include <_ID.h>

#include "chessboard.h"
#include "Skeletonize.h"
#include "Mapping.h"
#include "HeadTracker.h"
#include "FootDetection.h"

int main(int argc, char* argv[]){
	int result = -1;

	//result = main_chessboard(argc,argv);
	//result = main_skeletonize_net(argc,argv);
	//result = main_skeletonize(argc,argv);
	//result = main_mapping(argc,argv);
	//result = main_head_tracker(argc,argv);
	//result = main_foot_detection(argc,argv);

#ifdef _CCG
	result = main_chessboard_orig(argc,argv);
#endif

#ifdef _HOME
	result = main_skeletonize(argc,argv);
#endif

	return 0;
}