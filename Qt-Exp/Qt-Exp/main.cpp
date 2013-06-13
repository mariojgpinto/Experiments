#include "..\ThreadedGUI\main_gui.h"
#include <boost\thread.hpp>
#include <Windows.h>

int main(int argc, char* argv[]){
	boost::thread gui_thread(main_gui,argc,argv);

	while(true){
		printf("Main sleep\n");
		Sleep(1000);
	}

	gui_thread.join();

	return 0;
}