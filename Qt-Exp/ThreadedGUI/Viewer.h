#ifndef VIEWER_H
#define VIEWER_H

#include <QtGui/QMainWindow>

namespace Ui {
    class ViewerClass;//: public Ui_ViewerClass {};
}
//#include <ntk/ntk.h>
#include "controller.h"
#include "Preferences.h"

#include <ToolBoxQT.h>
//#include <ntk/gui/image_widget.h>

class Viewer : public QMainWindow
{
	Q_OBJECT

public:
	Viewer(Controller *c, QApplication *a, QWidget *parent = 0, Qt::WFlags flags = 0);
	~Viewer();

	
	ToolBoxQT::CVWidget *_ntk_widget_right;
	ToolBoxQT::CVWidget *_ntk_widget_left;

	void add_kinect(NIKinect* kinect);

	bool flag_color;
public slots:
    //Update Slot Method
    void update_window();

	void on_preferences_gui();

	//void on_push_button1();

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
	NIKinect *kinect;

	Controller* _controller;

	//Windows
	
	

	//timer
	double _last_tick;
	int _frame_counter;
	float _frame_rate;
};

#endif // VIEWER_H
