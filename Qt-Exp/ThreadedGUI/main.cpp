#include "Viewer.h"
#include <QtGui/QApplication>

#include "main_gui.h"

#include <boost\thread.hpp>

int main(int argc, char *argv[])
{
	//QApplication a(argc, argv);
	////getchar();

	//QNIKinect kinect("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni");

	//kinect.get_kinect()->set_processing_flag(NIKinect::DEPTH_COLOR, true);
	////QKinect kinect("C:\\x.oni");

	//Controller controller(&kinect);

	//Preferences preferences(&controller);
	//Viewer viewer(&controller, &a);
	//viewer.show();
	//return a.exec();

	boost::thread gui_thread(main_gui,argc,argv);

	while(true){
		printf("Main sleep");
		Sleep(1000);
	}

	gui_thread.join();

	return 0;
}
