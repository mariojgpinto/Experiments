#ifndef VIEWER_H
#define VIEWER_H

#include <QtGui/QMainWindow>
#include "ui_Viewer.h"

#include <ntk/ntk.h>
#include "controller.h"
#include "Preferences.h"

#include <ntk/gui/image_widget.h>

class Viewer : public QMainWindow
{
	Q_OBJECT

public:
	Viewer(Controller *c, QApplication *a, QWidget *parent = 0, Qt::WFlags flags = 0);
	~Viewer();

public slots:
    //Update Slot Method
    void update_window(ntk::RGBDImage* image);

	void on_preferences_gui();

	void on_close();

protected:
    //Setup Methods
    void setup_windows();
    void setup_connections();

    //Processing Methods
    void process_image();
    void show_images();

	void update_timer();

private:
	Ui::ViewerClass *ui;
	QApplication *app;


	Controller* _controller;

	//Windows
	ntk::ImageWidget *_ntk_widget_left;
	ntk::ImageWidget *_ntk_widget_right;

	//timer
	double _last_tick;
	int _frame_counter;
	float _frame_rate;
};

#endif // VIEWER_H
