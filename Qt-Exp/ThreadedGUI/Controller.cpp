#include "controller.h"
#include "Viewer.h"

void compute_color_encoded_depth2(const cv::Mat1f& depth_im, cv::Mat& color_depth_im,
                                     double* i_min_val, double* i_max_val){
	double min_val, max_val;
	if (i_min_val && i_max_val)
	{
		min_val = *i_min_val;
		max_val = *i_max_val;
	}
	else
	{
		minMaxLoc(depth_im, &min_val, &max_val);
	}

	color_depth_im.create(depth_im.size(),CV_8UC3);
	for (int r = 0; r < depth_im.rows; ++r)
	{
		const float* depth_data = depth_im.ptr<float>(r);
		cv::Vec3b* depth_color_data = color_depth_im.ptr<cv::Vec3b>(r);
		for (int c = 0; c < depth_im.cols; ++c)
		{
			int v = 255*6*(depth_data[c]-min_val)/(max_val-min_val);
			if (v < 0) v = 0;
			char r,g,b;
			int lb = v & 0xff;
			switch (v / 256) {
			case 0:
				r = 255;	g = 255-lb;	b = 255-lb;
				break;
			case 1:
				r = 255;	g = lb;		b = 0;
				break;
			case 2:
				r = 255-lb;	g = 255;	b = 0;
				break;
			case 3:
				r = 0;		g = 255;	b = lb;
				break;
			case 4:
				r = 0;		g = 255-lb;	b = 255;
				break;
			case 5:
				r = 0;		g = 0;		b = 255-lb;
				break;
			default:
				r = 0;		g = 0;		b = 0;
				break;
			}
			if (v == 0){
				r = g = b = 0;
			}
			depth_color_data[c] = cv::Vec3b(b,g,r);
		}
	}
}


//------------------------------------------------
// CONSTRUCTORS
//------------------------------------------------
Controller::Controller(/*QNIKinect* k*/){
    //this->_kinect = k;

    this->_paused = false;

    this->setup_images();
    this->setup_variables();
}

Controller::~Controller(){

}

//------------------------------------------------
// SETUP
//------------------------------------------------

/*
 * Initialize the image variables
 */
void Controller::setup_images(){
    //Retrieves the first image from kinect for initializations
    //this->_kinect_image = _kinect->get_image();

    //Initialize and create auxiliar images
    //this->_depth_image = new cv::Mat1f(640,480);
    //this->_color_image = new cv::Mat3b(640,480);
	//this->_depth_as_color.create(640,480);
    //this->_depth_as_color = new cv::Mat3b(640,480);
    this->_depth_mask = new cv::Mat1b(640,480);
    this->_diff = new cv::Mat1b(640,480);

    //this->_kinect_image->depth().copyTo(*this->_depth_image);
    //this->_kinect_image->rgb().copyTo(*this->_color_image);
    //this->_kinect_image->depthMask().copyTo(*this->_depth_mask);

    //ntk::compute_color_encoded_depth(this->_kinect_image->depth(), *this->_depth_as_color, &this->_min_depth, &this->_max_depth);
}

/*
 * Initialize the global variables
 */
void Controller::setup_variables(){
    this->_min_depth = 0.5;
    this->_max_depth = 1.0;
}

//------------------------------------------------
// SETUP
//------------------------------------------------
/*
 *
 */
void Controller::update(){
    if(!this->_paused){
        //Retrieves the first image from kinect for initializations
//        image->copyTo(*this->_kinect_image_temp);
//        this->_kinect_image_temp->swap(*thi   s->_kinect_image);
        //this->_kinect->acquire_read_lock();
        //image->copyTo(*this->_kinect_image);


        ////Copy the auxiliar images;
        //this->_kinect_image->depth().copyTo(*this->_depth_image);
        //this->_kinect_image->depthMask().copyTo(*this->_depth_mask);
        //this->_kinect_image->rgb().copyTo(*this->_color_image);
        ////this->_kinect_image->depthMask().copyTo(*this->_diff);

        //ntk::compute_color_encoded_depth(*this->_depth_image, *this->_depth_as_color, &this->_min_depth, &this->_max_depth);

		//this->_kinect->get_kinect()->get_color(this->_color_image);
		//this->_kinect->get_kinect()->get_depth(this->_depth_image);
		//this->_kinect->get_kinect()->get_depth_as_color(this->_depth_as_color);
		//double min = 400, max = 1500;
		//compute_color_encoded_depth2((cv::Mat1f)_depth_image,(cv::Mat3b)_depth_as_color,&min,&max);
		

        this->process_images();

        //this->_kinect->release_read_lock();
    }
}

/**
 *
 */
void Controller::process_images(){

}

/**
 *
 */
void Controller::show_images(){
	if(!this->_preferences_window->isHidden()){
		this->_preferences_window->show_images();
	}
}