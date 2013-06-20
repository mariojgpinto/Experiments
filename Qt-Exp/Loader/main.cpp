
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

	MainGUI *gui = new MainGUI(argc,argv);
	
	//NIKinect* kinect = new NIKinect();
	//kinect->set_processing_flag(NIKinect::DEPTH_COLOR, true);

	//kinect->init("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni");

	//boost::thread kinect_thread(&NIKinect::run, kinect);

	//gui->add_kinect(kinect);
	
	boost::thread gui_thread(&MainGUI::run, gui,argc,argv);
	Sleep(200);

	cv::Mat3b dc;
	
	while(true){
		printf("Main sleep\n");
		//kinect->get_depth_as_color(dc);

		//cv::imshow("image",dc);
		//cv::waitKey(100);
		Sleep(1009);
		//gui->update();
	}

	gui_thread.join();

	printf("End\n");
	Sleep(10000);

	return 0;
}