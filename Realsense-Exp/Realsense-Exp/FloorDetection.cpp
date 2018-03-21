#pragma warning(disable:4996)

#include "FloorDetection.h"

#include "SharedFunctions.h"

#include <stdio.h>


#include <Macros\DepthSensorMacros.cs>
#include <DepthSensorDLL.h>
#include <RealSense.h>

#include <opencv2\opencv.hpp>
#include <time.h>

#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace cv;
using namespace std;

bool calc_plane_from_points(std::vector<cv::Point3f*>* points, double* a, double* b, double* c, double* d, double dist_threshold) {
	if (!points || points->size() < 3) return false;

	pcl::PointCloud<pcl::PointXYZ> cloud;
	//Fill in the cloud data
	cloud.width = points->size();
	cloud.height = 1;
	cloud.points.resize(cloud.width * cloud.height);

	for (int i = 0; i < points->size(); ++i) {
		cloud[i] = pcl::PointXYZ(points->at(i)->x, points->at(i)->y, points->at(i)->z);
	}

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_PROSAC);
	seg.setDistanceThreshold(dist_threshold);

	seg.setInputCloud(cloud.makeShared());
	seg.segment(*inliers, *coefficients);

	int s = coefficients->values.size();

	if (!s) return false;

	*a = coefficients->values[0];
	*b = coefficients->values[1];
	*c = coefficients->values[2];
	*d = coefficients->values[3];

	std::cerr << "Model coefficients: "	<< coefficients->values[0] << " " 
										<< coefficients->values[1] << " "
										<< coefficients->values[2] << " " 
										<< coefficients->values[3] << std::endl;

	return true;
}

float DistanceToPlane(float pt_x, float pt_y, float pt_z, float a, float b, float c, float d) {
	float v = a*pt_x + b*pt_y + c*pt_z + d;
	v /= sqrt(a*a + b*b + c*c);
	return abs(v);
}

int main_FloorDetection(int argc, char* argv[]) {
	std::cout << "main_FloorDetection" << std::endl;
	std::cout << SensorVersion() << std::endl;

	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

	int depthWidth = 640;
	int depthHeight = 360;

	int deviceController = DEVICES::REALSENSE;

	int nDevices = GetDeviceCounter(deviceController);
	std::cout << "N Devices: " << nDevices << std::endl;

	CreateWindows_RealsenseKinects(nDevices);

	uchar** buffDepth = (uchar**)malloc(sizeof(uchar*) * nDevices);
	cv::Mat** depthImage = (cv::Mat**)malloc(sizeof(cv::Mat*) * nDevices);

	uchar** buffDepthMask = (uchar**)malloc(sizeof(uchar*) * nDevices);
	cv::Mat** depthImageMask = (cv::Mat**)malloc(sizeof(cv::Mat*) * nDevices);

	for (int i = 0; i < nDevices; ++i) {
		int initSensor = InitializeSensor(deviceController, i);

		//GetImageSize(SOURCES::DEPTH_RAW, &depthWidth, &depthHeight, 0, i);
		int returnSourceDepth = InitializeSource(SOURCES::DEPTH, depthWidth, depthHeight, i);

		int setCalibration = SetSensorCalibration("C:\\Prozis\\Data\\System\\Conf_HighDensity.json", i);

		buffDepth[i] = (uchar*)malloc(sizeof(uchar) * depthWidth * depthHeight * 1);
		depthImage[i] = new cv::Mat(depthHeight, depthWidth, CV_8UC1);

		buffDepthMask[i] = (uchar*)malloc(sizeof(uchar) * depthWidth * depthHeight * 1);
		depthImageMask[i] = new cv::Mat(depthHeight, depthWidth, CV_8UC1);
	}

	for (int i = 0; i < nDevices; ++i) {
		int startSensor = StartSensor(i);
	}

	int x_delta = 225;
	int y_delta = 75;
	int x_max = 350;
	int y_max = 150;

	int x_mean = depthWidth / 2;
	int y_mean = depthHeight / 2;;

	bool hasPlane = false;
	double plane_a = 0;
	double plane_b = 0;
	double plane_c = 0;
	double plane_d = 0;

	while (1) {
		for (int i = 0; i < nDevices; ++i) {
			UpdateSensor(i);

			if (GetImage(SOURCES::DEPTH, buffDepth[i], i) == 0) {
				depthImage[i]->data = buffDepth[i];

				cv::rectangle(*depthImage[i], cv::Rect(x_delta, y_delta, x_max, y_max), cv::Scalar(255, 255, 255));
				
				cv::imshow(depthWindowNameRealsense[i], *depthImage[i]);
			}

			if (GetImage(SOURCES::DEPTH_MASK, buffDepthMask[i], i) == 0) {
				depthImageMask[i]->data = buffDepthMask[i];
				cv::imshow(maskWindowNameRealsense[i], *depthImageMask[i]);
			}
		}

		++_frame_counter;
		if (_frame_counter == 60)
		{
			double current_tick = cv::getTickCount();
			_frame_rate = _frame_counter / ((current_tick - _last_tick) / cv::getTickFrequency());
			_last_tick = current_tick;
			_frame_counter = 0;
			printf("FrameRate %.2f\n", _frame_rate);
		}

		char c = cv::waitKey(1);

		if (c == 27) {
			break;
		}

		if (hasPlane) {
			RealSense* sensor = (RealSense*)GetDepthSensorInstance(0);
			float xx = NAN, yy = NAN, zz = NAN;
			int result = 0;
			float distance = -1;
			float max_distance = -1;
			int max_xx = -1;
			int max_yy = -1;
			uchar* ptr = depthImageMask[0]->data;

			auto vertices = sensor->vertices;

			for (int x = 0; x < depthWidth; ++x) {
				for (int y = 0; y < depthHeight; ++y) {
					if (ptr[y * depthWidth + x]) {
						result = 0;
						//xx = NAN, yy = NAN, zz = NAN;

						result = sensor->GetPointIn3D(x, y, &xx, &yy, &zz);

						//rs2::vertex point = vertices[depthWidth * y + x];

						if (result == RETURN::SUCCESS && xx != 0 && yy != 0 && zz != 0) {
							distance = abs(DistanceToPlane(xx, yy, zz, plane_a, plane_b, plane_c, plane_d));

						//if (result == RETURN::SUCCESS && point.x != 0 && point.y != 0 && point.z != 0) {
							//distance = abs(DistanceToPlane(point.x, point.y, point.z, plane_a, plane_b, plane_c, plane_d));

							if (distance > max_distance) {
								max_distance = distance;
								max_xx = x;
								max_yy = y;
							}

							if (distance < .01f) {
								ptr[y * depthWidth + x] = 0;
							}
						}
					}
				}
			}

			if (max_xx > 0 && max_yy > 0) {
				cv::circle(*depthImage[0], cv::Point(max_xx, max_yy), 3, cv::Scalar(0, 0, 0), -1);
				char buff[32];
				sprintf_s(buff, "Dist: %.3f\0", max_distance);
				cv::rectangle(*depthImage[0], cv::Rect(30, 10, 120, 30), cv::Scalar(0,0,0), -1);
				putText(*depthImage[0], buff, cvPoint(30, 30),
					FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255, 255, 255), 1, CV_AA);

				result = sensor->GetPointIn3D(max_xx, max_yy, &xx, &yy, &zz);

				if (result == RETURN::SUCCESS && xx != 0 && yy != 0 && zz != 0) {
					distance = abs(DistanceToPlane(xx, yy, zz, plane_a, plane_b, plane_c, plane_d));

					printf("");
				}

			}
			cv::imshow("Everest4", (*depthImage[0]) & (*depthImageMask[0]));
		}

		if (c == ' ') {
			printf("Calc Plane");
			//bool calc_plane_from_points(std::vector<cv::Point3f*>* points, double* a, double* b, double* c, double* d, double dist_threshold) {
			std::vector<cv::Point3f*> points;

			RealSense* sensor = (RealSense*)GetDepthSensorInstance(0);
			float xx = NAN, yy = NAN, zz = NAN;
			int result = 0;
			for (int x = 0; x < x_max; ++x) {
				for (int y = 0; y < y_max; ++y) {
					result = 0;
					xx = NAN, yy = NAN, zz = NAN;

					result = sensor->GetPointIn3D(x_delta + x, y_delta + y, &xx, &yy, &zz);

					if (result == RETURN::SUCCESS && xx != NAN && yy != NAN && zz != NAN) {
						points.push_back(new cv::Point3f(xx, yy, zz));
					}
				}
			}


			bool result2 = calc_plane_from_points(&points, &plane_a, &plane_b, &plane_c, &plane_d, .1f);

			std::cout << "Plane: "	<< plane_a << " "
									<< plane_b << " "
									<< plane_c << " "
									<< plane_d << std::endl;
			hasPlane = true;
			points.clear();
			points.~vector();
		}

		if (c == 'p') {
			float xx = NAN, yy = NAN, zz = NAN;
			float distance = 0;
			RealSense* sensor = (RealSense*)GetDepthSensorInstance(0);
			int result = sensor->GetPointIn3D(x_mean, y_mean, &xx, &yy, &zz);

			if (result == RETURN::SUCCESS) {
				distance = DistanceToPlane(xx, yy, zz, plane_a, plane_b, plane_c, plane_d);
				std::cout << "Distance: " << distance << std::endl;
			}
		}
	}

	for (int i = 0; i < nDevices; ++i) {
		CloseSensor(i);
	}

	return 0;
}