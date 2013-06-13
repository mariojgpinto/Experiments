#include "loader.h"
#include <QtGui/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	Loader w;
	w.show();
	return a.exec();
}
