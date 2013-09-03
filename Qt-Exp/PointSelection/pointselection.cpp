#include "pointselection.h"

#include <QMouseEvent>

#include <ToolBoxCV.h>


PointSelection::PointSelection(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	ui.setupUi(this);
	
	QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
    sizePolicy1.setHorizontalStretch(0);
    sizePolicy1.setVerticalStretch(0);
    sizePolicy1.setHeightForWidth(true);

	this->_widget = new ToolBoxQT::CVWidget(this->ui.widget);
    this->_widget->setObjectName(QString::fromUtf8("Widget"));
    this->_widget->setSizePolicy(sizePolicy1);
	this->_widget->setFixedSize(this->ui.widget->width(),this->ui.widget->height());

	this->_image_orig = cv::imread("img.png");
	this->_image = cv::imread("img.png");

	this->_widget->setImage(&this->_image);

	this->_points = new std::vector<cv::Point>();

	connect(this->ui.pushButton_save, SIGNAL(clicked()), this, SLOT(on_save()));
	connect(this->ui.pushButton_reset, SIGNAL(clicked()), this, SLOT(on_reset()));
	
	//connect(this->ui.widget,SIGNAL(clicked()),this,SLOT(mousePressEvent(QGraphicsSceneMouseEvent *event)));
}

PointSelection::~PointSelection()
{

}

void PointSelection::on_save(){

	this->hide();
	exit(0);
}

void PointSelection::on_reset(){
	_image_orig.copyTo(this->_image);

	cv::Point *pointsi = (cv::Point*)malloc(sizeof(cv::Point));

	for(unsigned int i = 0 ; i < this->_points->size() ; i++){
		cv::Point point = this->_points->at(i);
		if(ToolBoxCV::in_range(&point)){
			pointsi[i] = cv::Point(this->_points->at(i));
			//this->area_max_min(points->at(i));
		}
	}

	const cv::Point* countours[1]={
        pointsi,
    };

	const int countours_n[1]={
        this->_points->size(),      
    };

	printf("Mask Creation\n");

	cv::Mat1b mask(480,640,(const uchar)0);
	bool ok = true;
	try{
		printf("Start cv::FillPoly\n");
		cv::fillPoly(mask,countours,countours_n,1,cv::Scalar(255,255,255));
		printf("End cv::FillPoly\n");
	}
	catch(...){
		printf("Error cv::FillPoly\n");
		ok = false;
	}


	this->_points->clear();
	this->_widget->setImage(&mask);
}

void PointSelection::mousePressEvent(QMouseEvent *event){
	QPoint pos = this->ui.widget->mapFrom(this->ui.centralWidget,event->pos());

	cv::circle(this->_image,cv::Point(pos.x(),pos.y()),5,cv::Scalar(0,0,255),-1);
	this->_points->push_back(cv::Point(pos.x(),pos.y()));

	this->_widget->setImage(&this->_image);
}