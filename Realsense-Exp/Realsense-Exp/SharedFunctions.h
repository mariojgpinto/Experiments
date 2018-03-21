#ifndef _SHARED_FUNCTIONS
#define _SHARED_FUNCTIONS

#include <string>
#include <fstream>
#include <streambuf>

#include <librealsense2\rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

#include <librealsense2/rs_advanced_mode.hpp>


static std::string get_device_name(const rs2::device& dev);
rs2::device get_a_realsense_device();
float get_depth_scale(rs2::device dev);
void CreateWindows_RealsenseKinects(int nKinects);


extern std::string depthWindowNameRealsense[];// = { "Depth0", "Depht1", "Depht2", "Depht3" };
extern std::string maskWindowNameRealsense[];// = { "Mask0", "Mask1", "Mask2", "Mask3" };
extern std::string colorWindowNameRealsense[];// = { "Color0", "Color1", "Color2", "Color3" };

#endif