#include <_ID.h>

#include "chessboard.h"
#include "Skeletonize.h"
#include "Mapping.h"
#include "HeadTracker.h"
#include "FootDetection.h"
#include "FullScreen.h"
#include "VideoRecorder.h"
#include "ChromaKey.h"

int main(int argc, char* argv[]){
	int result = -1;

	//result = main_chessboard(argc,argv);
	//result = main_skeletonize_net(argc,argv);
	//result = main_skeletonize(argc,argv);
	//result = main_mapping(argc,argv);
	//result = main_head_tracker(argc,argv);
	//result = main_foot_detection(argc,argv);
	//result = main_full_screen(argc,argv);
	//result = main_chroma_key(argc,argv);

#ifdef _CCG
	result = main_chroma_key(argc, argv);
#endif

#ifdef _HOME
	result = main_chroma_key(argc,argv);
#endif

	return 0;
}