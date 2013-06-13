
//#include <QtCore/QCoreApplication>

//int main(int argc, char *argv[])
//{
	//QCoreApplication a(argc, argv);

	//return a.exec();
//}


#include "..\ThreadedGUI\main_gui.h"
#include <boost\thread.hpp>
#include <Windows.h>

int main(int argc, char* argv[]){
	//boost::thread gui_thread(main_gui,argc,argv);

	MainGUI gui(argc,argv);
	boost::thread gui_thread(&MainGUI::run, gui,argc,argv);
	//while(true){
	//	printf("Main sleep\n");
	//	Sleep(1000);
	//}

	gui_thread.join();

	printf("End\n");
	Sleep(10000);

	return 0;
}