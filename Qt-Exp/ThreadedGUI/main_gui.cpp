#include "main_gui.h"
#include "Viewer.h"
#include <QtGui/QApplication>
#include <boost\thread.hpp>

int main_gui(int argc, char *argv[])
{
	QApplication a(argc, argv);
	//getchar();

	QNIKinect kinect("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni");

	kinect.get_kinect()->set_processing_flag(NIKinect::DEPTH_COLOR, true);
	//QKinect kinect("C:\\x.oni");

	Controller controller(&kinect);

	Preferences preferences(&controller);
	Viewer viewer(&controller, &a);
	viewer.show();
	return a.exec();
}

MainGUI::MainGUI(int argc, char** argv) {
	
}

MainGUI::~MainGUI(){

}

void MainGUI::run(int argc, char** argv){
	a = new QApplication(argc,argv);
	kinect = new QNIKinect("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni");
	kinect->get_kinect()->set_processing_flag(NIKinect::DEPTH_COLOR, true);

	controller = new Controller(kinect);

	preferences = new Preferences(controller);
	viewer = new Viewer(controller,a);
	
	viewer->show();
	a->exec();
}