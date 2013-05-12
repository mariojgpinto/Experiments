#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/filter.h>

//class SimpleOpenNIProcessor{
//	public:
//		SimpleOpenNIProcessor ();
//
//		void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
//		void normalEstimation(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
//
//		void run ();
//
//		pcl::visualization::PCLVisualizer viewer;
//		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
//		pcl::PointCloud<pcl::PointXYZ>::ConstPtr g_cloud; 
//        boost::mutex mtx_;
//};

class SimpleOpenNIViewer 
 { 
   public: 
     SimpleOpenNIViewer () ;
	 void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) ;
	 void run () ;

	 pcl::visualization::PCLVisualizer viewer; 
         pcl::PointCloud<pcl::PointXYZ>::ConstPtr g_cloud; 
		 pcl::PointCloud<pcl::PointXYZ>::ConstPtr g_cloud2; 
         boost::mutex mtx_; 

};