#include "Viewer.h"
#include <QtGui/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);

	QKinect kinect("C:\\Dev\\Kinect\\Data\\ONI\\mirror_2_box.oni");
	//QKinect kinect("C:\\x.oni");

	Controller controller(&kinect);

	Preferences preferences(&controller);
	Viewer viewer(&controller, &a);
	viewer.show();
	return a.exec();
}
