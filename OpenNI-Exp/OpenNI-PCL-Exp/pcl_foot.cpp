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


void compute_color_encoded_depth(const cv::Mat1f& depth_im, cv::Mat3b& color_depth_im,
                                     double* i_min_val, double* i_max_val);
float distanceToPlane(const XnPoint3D& p, float a, float b, float c, float d);

int main_pcl_foot(int argc, char* argv[]){
	bool result = false;
	NIKinect* kinect = new NIKinect();
	
	//result = kinect->init("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni");
	kinect->init();
	

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


	float a = 0,b = 0,c = 0,d = 0;
	float a2 = 0,b2 = 0,c2 = 0,d2 = 0;

	XnStatus rc;

	int _min_bar = 400;
	int _max_bar = 1200;
	int _thresh = 175;
	int _thresh_floor = 185;
	int _floor_range = 10;
	int _kernel = 3;
	cv::namedWindow("Ranged Image");
	cv::createTrackbar("MinDepth", "Ranged Image", &_min_bar, 5000, NULL);
	cv::createTrackbar("MaxDepth", "Ranged Image", &_max_bar, 5000, NULL);
	cv::createTrackbar("FloorHeight", "Ranged Image", &_thresh, 400, NULL);
	cv::createTrackbar("FloorThresh", "Ranged Image", &_floor_range, 200, NULL);
	cv::createTrackbar("Kernel", "Ranged Image", &_kernel, 11, NULL);


	pcl::visualization::CloudViewer viewer("Simple cloud_file Viewer");
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	cloud.width = 640*480;
	cloud.height = 1;
	cloud.points.resize (cloud.width * cloud.height);

	char ch = 0;

	while((ch = cv::waitKey(31)) != 27){
		if(!kinect->update()) 
			break;
		
		result = kinect->get_depth(depthMat16UC1);
		result = kinect->get_color(color);
		result = kinect->get_depth_meta_data(_depth_md);

		XnPoint3D point2;
		
		uint8_t* ptr_clr = (uint8_t*)color.data;

		cv::Mat mask_cv;			
		cv::inRange(depthMat16UC1,_min_bar,_max_bar,mask_cv);

		//depthMat16UC1.convertTo(depthMat8UC1, CV_8UC1,0.1);
		//cv::erode(depthMat8UC1,depthMat8UC1,cv::Mat(5,5,CV_8UC1));			
		//cv::dilate(depthMat8UC1,depthMat8UC1,cv::Mat(11,11,CV_8UC1));
		//cv::erode(depthMat8UC1,depthMat8UC1,cv::Mat(5,5,CV_8UC1));
		//
		//
		//uint8_t* ptr_depth = depthMat8UC1.data;
		
		uchar* ptr = mask_cv.data;

		for(int y=0; y<XN_VGA_Y_RES; y++) { 
			for(int x=0; x<XN_VGA_X_RES; x++) { 
				//if(x > 300 && x < 350 && y > 150 && y < 200){
				//	ptr_depth[y * XN_VGA_X_RES + x] = ptr_depth[y * XN_VGA_X_RES + x] * 32; 
				//}

				XnPoint3D point1;
				point1.X = x; 
				point1.Y = y; 
				//point1.Z = ptr_depth[y * XN_VGA_X_RES + x]; 
				point1.Z = _depth_md[y * XN_VGA_X_RES + x]; 

				pointList[y * XN_VGA_X_RES + x] = point1;
			}
		} 
		//cv::imshow("Depth",depthMat8UC1);
		_depth.ConvertProjectiveToRealWorld(XN_VGA_Y_RES*XN_VGA_X_RES, pointList, realWorld); 

		if(remove_floor){


			_thresh_floor = _thresh + _floor_range;

			floorPoint = floorCoords.ptPoint;
			floorPoint.X += floorCoords.vNormal.X * _thresh_floor;
			floorPoint.Y += floorCoords.vNormal.Y * _thresh_floor;
			floorPoint.Z += floorCoords.vNormal.Z * _thresh_floor;
			//floorPoint.Z -= _thresh_floor;
			a2 = floorCoords.vNormal.X;
			b2 = floorCoords.vNormal.Y;
			c2 = floorCoords.vNormal.Z;
			d2 = -(a2*floorPoint.X + b2*floorPoint.Y + c2*floorPoint.Z);

			for(int y=0; y<XN_VGA_Y_RES; y++) { 
				for(int x=0; x<XN_VGA_X_RES; x++) { 
					if(realWorld[y * XN_VGA_X_RES + x].Z > 0.0 /*&& ptr[y * XN_VGA_X_RES + x]*/){
						float value1 = distanceToPlane(realWorld[y * XN_VGA_X_RES + x],a,b,c,d) ;
						float value2 = distanceToPlane(realWorld[y * XN_VGA_X_RES + x],a2,b2,c2,d2) ;
						if( value1 < _thresh &&  value2 < _thresh){
							ptr[y * XN_VGA_X_RES + x]=255;
						}
						else{
							ptr[y * XN_VGA_X_RES + x]=0;
						}
					}
				} 
			}
		}

		cloud.points.clear();
		cv::imshow("Mask",mask_cv);

		for(int y=0; y<XN_VGA_Y_RES; y++) { 
			for(int x=0; x<XN_VGA_X_RES; x++) { 
				if(ptr[y * XN_VGA_X_RES + x]){
					pcl::PointXYZRGB pt(ptr_clr[y * XN_VGA_X_RES * 3 + x* 3 + 2],
										ptr_clr[y * XN_VGA_X_RES * 3 + x* 3 + 1],
										ptr_clr[y * XN_VGA_X_RES * 3 + x* 3 + 0]);

					pt.x = realWorld[y * XN_VGA_X_RES + x].X;
					pt.y = -realWorld[y * XN_VGA_X_RES + x].Y;
					pt.z = realWorld[y * XN_VGA_X_RES + x].Z;
					cloud.push_back(pcl::PointXYZRGB(pt));
				}
			}
		}

		viewer.showCloud(cloud.makeShared());

		cv::Mat3b cor(480,640);
		cv::Mat color3;
		double min = _min_bar,max = _max_bar;
		compute_color_encoded_depth(depthMat16UC1,cor,&min,&max);

		cor.copyTo(color3,mask_cv);

		cv::imshow("Ranged Image",color3);

		printf("Frame Rate: %.2f\n",kinect->get_frame_rate());

		if(ch == 'f'){
			if(remove_floor){
				remove_floor = false;
			}
			else{
				rc = xn_scene.GetFloor( floorCoords);
				//rc = xnGetFloor(xn_scene, &floorCoords);
				if(rc == XN_STATUS_OK){
					std::cout << floorCoords.vNormal.X << " " << floorCoords.vNormal.Y << " " << floorCoords.vNormal.Z << std::endl;
					std::cout << floorCoords.ptPoint.X << " " << floorCoords.ptPoint.Y << " " << floorCoords.ptPoint.Z  << "\n" << std::endl;

					a = floorCoords.vNormal.X;
					b = floorCoords.vNormal.Y;
					c = floorCoords.vNormal.Z;

					XnPoint3D pt1; pt1.X = 320; pt1.Y = 350; pt1.Z = _depth_md[ 350*640+ 320 ];
					XnPoint3D pt2;
					XnPoint3D pt3;
					//_depth.ConvertProjectiveToRealWorld(1,&floorCoords.ptPoint,&pt2);
					//_depth.ConvertProjectiveToRealWorld(1,&pt1,&pt2);
					_depth.ConvertRealWorldToProjective(1,&floorCoords.ptPoint,&pt3);

					d = -(a*floorCoords.ptPoint.X + b*floorCoords.ptPoint.Y + c*floorCoords.ptPoint.Z);

					floorPoint = floorCoords.ptPoint;
					floorPoint.Z-=_thresh_floor;

					a2 = floorCoords.vNormal.X;
					b2 = floorCoords.vNormal.Y;
					c2 = floorCoords.vNormal.Z;

					//_depth.ConvertRealWorldToProjective(1,&floorPoint,&pt2);

					d2 = -(a2*floorPoint.X + b2*floorPoint.Y + c2*floorPoint.Z);

					remove_floor = true;
				}
				else
					printf("Read failed: %s\n", xnGetStatusString(rc));
			}
		}
	}

	return 0;
}