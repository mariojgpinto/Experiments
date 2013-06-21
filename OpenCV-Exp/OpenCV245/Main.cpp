#include <_ID.h>

#include "chessboard.h"
#include "Skeletonize.h"
#include "Mapping.h"

int main(int argc, char* argv[]){
	int result = -1;

	//result = main_chessboard(argc,argv);
	//result = main_skeletonize_net(argc,argv);
	//result = main_skeletonize(argc,argv);
	//result = main_mapping(argc,argv);

#ifdef _CCG
	result = main_mapping(argc,argv);
#endif

#ifdef _HOME
	result = main_skeletonize(argc,argv);
#endif

	return 0;
}