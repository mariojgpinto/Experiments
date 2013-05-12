#include "SimpleOpenNIProcessor.h"


    SimpleOpenNIViewer::SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {
	
	} 

     void SimpleOpenNIViewer::cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) 
     { 
		 static bool first = true;

		 if(first){
			 boost::mutex::scoped_lock lock (mtx_); 
				{
					g_cloud = cloud; 
					std::vector<int> vec;
				//	pcl::removeNaNFromPointCloud(g_cloud,g_cloud2,&vec);
					if(g_cloud){
						viewer.addPointCloud<pcl::PointXYZ> (g_cloud);
						first = false;
					}
				}
				
		 }
		 else{
			 boost::mutex::scoped_lock lock (mtx_); 
				{
					g_cloud = cloud; 
					if(g_cloud)
						viewer.updatePointCloud<pcl::PointXYZ> (g_cloud);
				}
		 }
                
     } 

     void SimpleOpenNIViewer::run () 
     { 
       pcl::Grabber* interface = new pcl::OpenNIGrabber(); 

       boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = 
         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1); 

       interface->registerCallback (f); 

       interface->start (); 
            

       while (!viewer.wasStopped()) 
       { 
/*                        boost::mutex::scoped_lock lock (mtx_); 
						if(g_cloud){
                        if(!viewer.updatePointCloud<pcl::PointXYZ>(g_cloud)) 
                                viewer.addPointCloud<pcl::PointXYZ> (g_cloud);*/ 
						//}
		   viewer.spinOnce (100); 
         pcl_sleep (1); 
       } 

       interface->stop (); 
     }


//
//SimpleOpenNIProcessor::SimpleOpenNIProcessor() : viewer ("PCL OpenNI Viewer"),cloud_normals (new pcl::PointCloud<pcl::Normal>){
//
//}
//
//void SimpleOpenNIProcessor::cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud){
//	static unsigned count = 0;
//	static double last = pcl::getTime ();
//
//	if (++count == 30){
//		double now = pcl::getTime ();
//		std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
//		count = 0;
//		last = now;
//	}
//
//	//this->normalEstimation(cloud);
//
//
//	if (!viewer.wasStopped()){
//		//viewer.showCloud(cloud);
//		
//		boost::mutex::scoped_lock lock (mtx_); 
//		g_cloud = cloud; 
//		viewer.addPointCloud<pcl::PointXYZ>(g_cloud);
//		viewer.spinOnce();
//	}
//}
//
//void SimpleOpenNIProcessor::run (){
//	// create a new grabber for OpenNI devices
//	pcl::Grabber* interface = new pcl::OpenNIGrabber();
//
//	// make callback function from member function
//	boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
//		boost::bind (&SimpleOpenNIProcessor::cloud_cb_, this, _1);
//
//	// connect callback function for desired signal. In this case its a point cloud with color values
//	boost::signals2::connection c = interface->registerCallback (f);
//
//	// start receiving point clouds
//	interface->start ();
//
//	// wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
//	while (true){
//		boost::this_thread::sleep (boost::posix_time::seconds (1));
//	}
//	// stop the grabber
//	interface->stop ();
//}
//
//void SimpleOpenNIProcessor::normalEstimation(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud){
//	// Create the normal estimation class, and pass the input dataset to it
//	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//
//	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
//	ne.setMaxDepthChangeFactor(0.02f);
//	ne.setNormalSmoothingSize(10.0f);
//	ne.setInputCloud(cloud);
//	ne.compute(*normals);
//
//	// cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
//}