#ifndef _SHARED_FUNCTIONS
#define _SHARED_FUNCTIONS

#include <string>
#include <fstream>
#include <streambuf>

#include <librealsense2\rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

#include <librealsense2/rs_advanced_mode.hpp>
using namespace cv;

static std::string get_device_name(const rs2::device& dev);
rs2::device get_a_realsense_device();
float get_depth_scale(rs2::device dev);

#endif