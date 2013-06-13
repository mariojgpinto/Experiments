#ifndef LOADER_H
#define LOADER_H

#include <QtGui/QMainWindow>
#include "ui_loader.h"

class Loader : public QMainWindow
{
	Q_OBJECT

public:
	Loader(QWidget *parent = 0, Qt::WFlags flags = 0);
	~Loader();

private:
	Ui::LoaderClass ui;
};

#endif // LOADER_H
