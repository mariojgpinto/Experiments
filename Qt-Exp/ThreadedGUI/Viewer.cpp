#include "Viewer.h"
#include "ui_Viewer.h"

Viewer::Viewer(Controller *c, QApplication *a, QWidget *parent, Qt::WFlags flags):
	QMainWindow(parent, flags),
	app(a),
	ui(new Ui::ViewerClass),
	_last_tick(0),
    _frame_counter(0),
    _frame_rate(0)
{
	ui->setupUi(this);

	this->_controller = c;
    this->_controller->set_viewer_window(this);

	this->_controller->set_paused(true);
    this->setup_windows();
    this->setup_connections();
    this->_controller->set_paused(false);

	flag_color = false;
}

Viewer::~Viewer()
{
	delete ui;
}

void Viewer::add_kinect(NIKinect* kinect){
	this->kinect = kinect;
}

//-----------------------------------------------------------------------------
// Setup
//-----------------------------------------------------------------------------
void Viewer::setup_windows(){
    QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
    sizePolicy1.setHorizontalStretch(0);
    sizePolicy1.setVerticalStretch(0);
    sizePolicy1.setHeightForWidth(true);

    this->_ntk_widget_left = new ToolBoxQT::CVWidget(this->ui->main_widget_left);
    this->_ntk_widget_left->setObjectName(QString::fromUtf8("Color"));
    this->_ntk_widget_left->setSizePolicy(sizePolicy1);
	this->_ntk_widget_left->setFixedSize(this->ui->main_widget_left->width(),this->ui->main_widget_left->height());

	this->_ntk_widget_right = new ToolBoxQT::CVWidget(this->ui->main_widget_right);
    this->_ntk_widget_right->setObjectName(QString::fromUtf8("DepthAsColor"));
    this->_ntk_widget_right->setSizePolicy(sizePolicy1);
	this->_ntk_widget_right->setFixedSize(this->ui->main_widget_right->width(),this->ui->main_widget_right->height());

    //int _min = (this->_controller->get_min_depth() - this->min_value) * 100 / this->max_value;
    //this->min_slider_change(_min);
    //this->ui->main_horizontalSlider_min_depth->setValue(_min);
    //int _max = (this->_controller->get_max_depth() - this->min_value) * 100 / this->max_value;
    //this->max_slider_change(_max);
    //this->ui->main_horizontalSlider_max_depth->setValue(_max);
}

void Viewer::setup_connections(){
    //connect(this->_controller->get_kinect(), SIGNAL(kinect_image()),this, SLOT(update_window()));

	this->ui->main_action_quit->setShortcut(Qt::Key_Escape);
	connect(this->ui->main_action_quit,SIGNAL(triggered()),this,SLOT(on_close()));

	this->ui->main_action_preferences->setShortcut(QKeySequence("Ctrl+P"));
	connect(this->ui->main_action_preferences,SIGNAL(triggered()),this,SLOT(on_preferences_gui()));

    //connect(this->ui->main_pushButton_floor, SIGNAL(clicked()), this, SLOT(on_botton_floor()));
    //connect(this->ui->main_pushButton_background, SIGNAL(clicked()), this, SLOT(on_botton_background()));
    //connect(this->ui->main_pushButton_user, SIGNAL(clicked()), this, SLOT(on_botton_user()));
    //connect(this->ui->main_horizontalSlider_min_depth, SIGNAL(valueChanged(int)), this, SLOT(min_slider_change(int)));
    //connect(this->ui->main_horizontalSlider_max_depth, SIGNAL(valueChanged(int)), this, SLOT(max_slider_change(int)));
}

//-----------------------------------------------------------------------------
// Slots
//-----------------------------------------------------------------------------
void Viewer::update_window(){
	//    this->_controller->get_kinect()->acquire_read_lock();

    this->_controller->update();

    this->process_image();

	//

	this->update_timer();
	
	this->show_images();

	//	this->_controller->get_kinect()->release_read_lock();

}

//bool _floor = false;

void Viewer::on_preferences_gui(){
	flag_color = !flag_color;
	this->_controller->get_preferences_window()->show();

	//if(!_floor){
	//	kinect->init_scene_analyzer();
	//	_floor = true;
	//}
	//else{
	//	double _a,_b,_c,_d;
	//	bool result = kinect->get_floor_plane(&_a,&_b,&_c,&_d);

	//	if(result){
	//		printf("Floor Plane (%.4d,%.4d,%.4d,%.4d)\n",_a,_b,_c,_d);
	//	}
	//	else{
	//		printf("No Floor\n");
	//	}
	//}
	
}

void Viewer::on_close(){
	this->app->quit();
	exit(0);
}

//-----------------------------------------------------------------------------
// Processing
//-----------------------------------------------------------------------------
void Viewer::process_image()
{
	this->_controller->process_images();
    //if(!this->_controller->get_user_window()->isHidden()){
    //    this->_controller->get_user_window()->process_image();
    //}
}

void Viewer::show_images()
{
	//this->_ntk_widget_right->setImage((cv::Mat3b)*this->_controller->get_color_image());
	//this->_ntk_widget_left->setImage((cv::Mat1f)*this->_controller->get_depth_image());
	//this->_ntk_widget_right->setImage(*this->_controller->get_depth_as_color());

	this->_controller->show_images();
}

void Viewer::update_timer(){
	++this->_frame_counter;
    if (this->_frame_counter == 15)
    {
        double current_tick = cv::getTickCount();
        this->_frame_rate = this->_frame_counter / ((current_tick - this->_last_tick)/cv::getTickFrequency());
        this->_last_tick = current_tick;
        this->_frame_counter = 0;
    }

	QString status = QString("FPS = %1").arg(this->_frame_rate, 0, 'f', 1);

	this->setWindowTitle(status);
}
