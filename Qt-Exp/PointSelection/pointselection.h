#ifndef POINTSELECTION_H
#define POINTSELECTION_H

#include <QtGui/QMainWindow>
#include "ui_pointselection.h"

#include <opencv2\opencv.hpp>

#include <ToolBoxQT.h>

class PointSelection : public QMainWindow
{
	Q_OBJECT

public:
	PointSelection(QWidget *parent = 0, Qt::WFlags flags = 0);
	~PointSelection();

public:
    void mousePressEvent(QMouseEvent *event);

public slots:
	void on_save();
	void on_reset();

private:
	Ui::PointSelectionClass ui;
	QAction* _close;

	ToolBoxQT::CVWidget* _widget;

	std::vector<cv::Point>* _points;

	cv::Mat _image_orig;
	cv::Mat _image;
};

#endif // POINTSELECTION_H
