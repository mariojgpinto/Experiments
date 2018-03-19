#include "PointCloud.h"
#include "Skeleton.h"

int main(int argc, char* argv[])
{
	int result = -1;
	//result = main_pcl_point_cloud_v1(argc, argv);
	//result = main_pcl_point_cloud_v2(argc, argv);
	//result = main_pcl_point_cloud_v3(argc, argv);
	//result = main_pcl_skeleton_v1(argc, argv);

	result = main_pcl_skeleton_v1(argc, argv);

	return result;
}

