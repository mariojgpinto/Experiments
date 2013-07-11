#include "Viewer3D.h"

Viewer3D::Viewer3D(NIThreadedKinect* kinect){
	kinect = kinect;
	cloud.width = 640*480;
	cloud.height = 1;
	cloud.points.resize (cloud.width * cloud.height);

	cloud_color.width = 640*480;
	cloud_color.height = 1;
	cloud_color.points.resize (cloud.width * cloud.height);

	viewer = new pcl::visualization::CloudViewer("Viewer3D");
}

Viewer3D::~Viewer3D(){

}