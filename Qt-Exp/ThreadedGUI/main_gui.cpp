#include "main_gui.h"
#include "Viewer.h"
#include <QtGui/QApplication>
#include <boost\thread.hpp>

#include <NIKinect.h>

int main_gui(int argc, char *argv[])
{
	//QApplication a(argc, argv);
	////getchar();

	//NIKinect kinect();//("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni");

	//kinect.set_processing_flag(NIKinect::DEPTH_COLOR, true);
	////QKinect kinect("C:\\x.oni");

	//Controller controller(&kinect);

	//Preferences preferences(&controller);
	//Viewer viewer(&controller, &a);
	//viewer.show();
	//return a.exec();
	return 0;
}

MainGUI::MainGUI(int argc, char** argv) {
	
}

MainGUI::~MainGUI(){

}

void MainGUI::update(){
	//this->viewer->update_window();
	//up = true;
	//cv::Mat3b color;
	//this->kinect->get_color(color);
	//this->viewer->_ntk_widget_left->setImage(color);
	//this->viewer->updateGeometry();
}

void MainGUI::update_cycle(){
	up = false;
	while(true){
		if(up){
			this->viewer->update_window();
			up = false;
		}
		Sleep(100);
	}
}

void MainGUI::add_kinect(NIKinect* kinect){
	//this->kinect = kinect;
}

void MainGUI::run(int argc, char** argv){
	a = new QApplication(argc,argv);

	kinect = new QNIKinect("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni");
	kinect->get_kinect()->set_processing_flag(NIKinect::DEPTH_COLOR, true);

	controller = new Controller(kinect);

	preferences = new Preferences(controller);
	viewer = new Viewer(controller,a);
	
	//boost::thread coiso(&MainGUI::update_cycle, this);

	viewer->show();
	a->exec();

}