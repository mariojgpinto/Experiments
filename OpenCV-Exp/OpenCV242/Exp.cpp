#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

using namespace cv;
using namespace std;

bool running = true;

//Data given from depth generator
Mat depth_map;			//CV_CAP_OPENNI_DEPTH_MAP			- depth values in mm (CV_16UC1)
Mat point_cloud_map;	//CV_CAP_OPENNI_POINT_CLOUD_MAP		- XYZ in meters (CV_32FC3)
Mat disparity_map;		//CV_CAP_OPENNI_DISPARITY_MAP		- disparity in pixels (CV_8UC1)
Mat disparity_map_32;	//CV_CAP_OPENNI_DISPARITY_MAP_32F	- disparity in pixels (CV_32FC1)
Mat valid_depth_map;	//CV_CAP_OPENNI_VALID_DEPTH_MASK	- mask of valid pixels (CV_8UC1)

//Data given from RGB image generator
Mat color_image;		//CV_CAP_OPENNI_BGR_IMAGE			- color image (CV_8UC3)
Mat gray_image;			//CV_CAP_OPENNI_GRAY_IMAGE			- gray image (CV_8UC1)

int main( int argc, char* argv[] ){
	VideoCapture capture;

	capture.open( CV_CAP_OPENNI );

	if( !capture.isOpened() ){
        cout << "Can not open a capture object." << endl;
        return -1;
    }

	while(running && capture.grab()){
		capture.retrieve(depth_map, CV_CAP_OPENNI_DEPTH_MAP);
		capture.retrieve(point_cloud_map, CV_CAP_OPENNI_POINT_CLOUD_MAP);
		capture.retrieve(disparity_map, CV_CAP_OPENNI_DISPARITY_MAP);
		capture.retrieve(disparity_map_32, CV_CAP_OPENNI_DISPARITY_MAP_32F);
		capture.retrieve(valid_depth_map, CV_CAP_OPENNI_VALID_DEPTH_MASK);

		capture.retrieve(color_image, CV_CAP_OPENNI_BGR_IMAGE);
		capture.retrieve(gray_image, CV_CAP_OPENNI_GRAY_IMAGE);

		CV_Assert(disparity_map_32.type() == CV_32FC1);
		const float scaleFactor = 0.05f;
        depth_map.convertTo( depth_map, CV_8UC1, scaleFactor );

		disparity_map_32.convertTo( disparity_map_32, CV_8UC1, scaleFactor );

		imshow("depth_map",depth_map);
		imshow("point_cloud_map",point_cloud_map);
		imshow("disparity_map",disparity_map);
		imshow("disparity_map_32",disparity_map_32);
		imshow("valid_depth_map",valid_depth_map);
		imshow("color_image",color_image);
		imshow("gray_image",gray_image);

		waitKey(1);
	}
}