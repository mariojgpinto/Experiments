#ifndef _VIEWER_3D
#define _VIEWER_3D

#include <boost\thread\thread.hpp>
#include <boost\thread\condition_variable.hpp>
#include <Windows.h>
#include <NIThreadedKinect.h>

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

class __declspec(dllexport) Viewer3D{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		Viewer3D(NIThreadedKinect* kinect);
		~Viewer3D();



		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::PointCloud<pcl::PointXYZRGB> cloud_color;
		pcl::visualization::CloudViewer* viewer;
		NIThreadedKinect* kinect;
};

#endif//_VIEWER_3D