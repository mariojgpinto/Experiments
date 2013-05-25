#ifndef PREFERENCES_GUI_H
#define PREFERENCES_GUI_H

#include <QtGui/QMainWindow>
#include "ui_Preferences.h"

#include <ntk/ntk.h>
#include <ntk/gui/image_widget.h>

#include "Controller.h"

class Preferences : public QMainWindow
{
	Q_OBJECT

	public:
		Preferences(Controller *c, QWidget *parent = 0, Qt::WFlags flags = 0);
		~Preferences();

	//Processing Methods
		void process_image();
		void show_images();

	public slots:
		void on_close();

		//Sliders
		void min_slider_change(int value);
		void max_slider_change(int value);

	protected:
		void setup_windows();
		void setup_connections();
		void setup_variables();

	private:
		Ui::Preferences *ui;
		QAction *_close;

		//Controller
		Controller *_controller;

		////Windows
		//ntk::ImageWidget *_widget_top_left;
		//ntk::ImageWidget *_widget_top_right;

		////Button Flags
		//bool _button_1;

		//Control Values
		double _min_value;
		double _max_value;
		//    double min_depth;
		//    double max_depth;
		double _min_depth_old;
		double _max_depth_old;

};

#endif // PREFERENCES_GUI_H