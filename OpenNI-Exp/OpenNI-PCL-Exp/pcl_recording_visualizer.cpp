#include "pcl_recording_visualizer.h"
#include <NIKinect.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

int main_pcl_recording_visualizer(int argc, char* argv[]){
	bool result = false;
	NIKinect* kinect = new NIKinect();
	
	result = kinect->init("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni");
	//kinect->init();
	

//	result = kinect->init_generators();

	cv::Mat color;
	cv::Mat depth;
	cv::Mat mask;
	cv::Mat depthMat8UC1;
	cv::Mat depthMat16UC1;
	cv::Mat depth_as_color;

	xn::DepthGenerator _depth = kinect->get_depth_generator();
	xn::SceneAnalyzer xn_scene = kinect->get_scene_analyzer();
	xn::DepthMetaData _depth_md;

	bool remove_floor = false;
	XnPlane3D floorCoords;
    XnPoint3D floorPoint;
	XnPoint3D * pointList = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 
	XnPoint3D * realWorld = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 
	pcl::visualization::CloudViewer viewer("Simple cloud_file Viewer");
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	cloud.width = 640*480;
	cloud.height = 1;
	cloud.points.resize (cloud.width * cloud.height);

	XnStatus rc;

	char c = 0;

	while((c = cv::waitKey(31)) != 27){
		if(!kinect->update()) 
			break;
		
		result = kinect->get_depth(depthMat16UC1);
		result = kinect->get_color(color);
		result = kinect->get_depth_meta_data(_depth_md);

		XnPoint3D point2;

		uchar* ptr_depth = depthMat16UC1.data;
		uint8_t* ptr_clr = (uint8_t*)color.data;

		for(int y=0; y<XN_VGA_Y_RES; y++) { 
			for(int x=0; x<XN_VGA_X_RES; x++) { 
				XnPoint3D point1;
				point1.X = x; 
				point1.Y = y; 
				point1.Z = kinect->_depth_md[y * XN_VGA_X_RES + x]; 

				pointList[y * XN_VGA_X_RES + x] = point1;
			}
		} 

		_depth.ConvertProjectiveToRealWorld(XN_VGA_Y_RES*XN_VGA_X_RES, pointList, realWorld); 

		cloud.points.clear();
		
		//for(int y=0; y<XN_VGA_Y_RES; y++) { 
		//	for(int x=0; x<XN_VGA_X_RES; x++) { 
		for(int y=50; y<XN_VGA_Y_RES-50; y++) { 
			for(int x=100; x<XN_VGA_X_RES-100; x++) { 
				if(realWorld[y * XN_VGA_X_RES + x].Z > 0.0){
					pcl::PointXYZRGB pt(ptr_clr[y * XN_VGA_X_RES * 3 + x* 3 + 2],
										ptr_clr[y * XN_VGA_X_RES * 3 + x* 3 + 1],
										ptr_clr[y * XN_VGA_X_RES * 3 + x* 3 + 0]);

					pt.x = realWorld[y * XN_VGA_X_RES + x].X;
					pt.y = -realWorld[y * XN_VGA_X_RES + x].Y;
					pt.z = realWorld[y * XN_VGA_X_RES + x].Z;
					cloud.push_back(pcl::PointXYZRGB(pt));
					//cloud.points[y * XN_VGA_X_RES + x] = pt;
				}
			} 
		}

		viewer.showCloud(cloud.makeShared());

		printf("Frame Rate: %.2f\n",kinect->get_frame_rate());
	}

	return 0;
}