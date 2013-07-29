#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
void write(){
	FILE* fp = fopen("file.pcd","w+");

	if(!fp) return;

	fprintf(fp,"# .PCD v0.7 - Point cloud_file Data file format\n");
	fprintf(fp,"VERSION 0.7\n");
	fprintf(fp,"FIELDS x y z\n");
	fprintf(fp,"SIZE 4 4 4\n");
	fprintf(fp,"TYPE F F F\n");
	fprintf(fp,"COUNT 1 1 1\n");
	fprintf(fp,"WIDTH 640\n");
	fprintf(fp,"HEIGHT 480\n");
	fprintf(fp,"VIEWPOINT 1 0 1 0 1 0 0\n");
		
	fprintf(fp,"POINTS 307200\n");
	fprintf(fp,"DATA ascii\n");

	int ac = 0;
	fprintf(fp,"0.0 0.0 0.0\n");ac++;
	for(float i = 0.0f ; i < 1.0f ; i+=0.1f){
		for(float j = 0.0f ; j < 1.0f ; j+=0.1f){
			fprintf(fp,"%f 0.2 %f\n",-1.0f-i,-1.0f-j);ac++;
		}
	}
	fprintf(fp,"0.0 0.0 0.0\n");ac++;

	for(int i = ac ; i < 307200 ; i++){
		fprintf(fp,"-1.5 0.0 -1.5\n");
	}

	fclose(fp);
}

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_file (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> cloud2;// (new pcl::PointCloud<pcl::PointXYZ>);
  
  // write();
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("file.pcd", *cloud_file) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud_file->width * cloud_file->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

 // 	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	//pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	//// Create the segmentation object
	//pcl::SACSegmentation<pcl::PointXYZ> seg;
	//// Optional
	//seg.setOptimizeCoefficients (true);
	//// Mandatory
	//seg.setModelType (pcl::SACMODEL_PLANE);
	//seg.setMethodType (pcl::SAC_RANSAC);
	//seg.setDistanceThreshold (0.01);

	//seg.setInputCloud (cloud_file->makeShared ());
	//seg.segment (*inliers, *coefficients);

	//double a = coefficients->values[0];
	//double b = coefficients->values[1];
	//double c = coefficients->values[2];
	//double d = coefficients->values[3];

	//printf("%d, %d, %d, %d\n",a,b,c,d);


	 pcl::PointCloud<pcl::PointXYZ> cloud;
	 cloud.width = cloud_file->size();
	 cloud.height = 1;
	 cloud.points.resize (cloud.width * cloud.height);


	 cloud2.width = cloud_file->size();
	 cloud2.height = 1;
	 cloud2.points.resize (cloud2.width * cloud2.height);

	 long long zzz = 0;
	 int ac = 0;
	 for(int i = 0 ; i < cloud_file->size() ; i++){
		 pcl::PointXYZ pt = cloud_file->at(i);

		 cloud.push_back(pcl::PointXYZ(pt));
		 zzz+=pt.z;
		 ac++;
		 cloud2.push_back(pcl::PointXYZ(pt.x/1000.0,pt.y/1000.0,pt.z/1000.0));
	 }

	 double res = zzz / ac;
  //// Fill in the cloud data
  //cloud.width  = 1500;
  //cloud.height = 1;
  //cloud.points.resize (cloud.width * cloud.height);

  //// Generate the data
  //for (size_t i = 0; i < cloud.points.size (); ++i)
  //{
  //  cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
  //  cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
  //  cloud.points[i].z = 1.0;
  //}

  //// Set a few outliers
  //cloud.points[0].z = 2.0;
  //cloud.points[3].z = -2.0;
  //cloud.points[6].z = 4.0;

  //std::cerr << "Point cloud data: " << cloud.points.size () << " points" << std::endl;
  ////for (size_t i = 0; i < cloud.points.size (); ++i)
  ////  std::cerr << "    " << cloud.points[i].x << " " 
  ////                      << cloud.points[i].y << " " 
  ////                      << cloud.points[i].z << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_PROSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud2.makeShared());
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  //std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  //for (size_t i = 0; i < inliers->indices.size (); ++i)
  //  std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
  //                                             << cloud.points[inliers->indices[i]].y << " "
  //                                             << cloud.points[inliers->indices[i]].z << std::endl;


  //pcl::VoxelGrid<pcl::PointXYZ> sor;
  //sor.setInputCloud(cloud_file);

  //  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  //sor.filter (*cloud2);


  
   //... populate cloud_file
	pcl::visualization::CloudViewer viewer("Simple cloud_file Viewer");
 
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
   viewer2->initCameraParameters ();
	
	viewer2->addPointCloud (cloud2.makeShared(), "cloud");
	viewer2->addCoordinateSystem (1.0);

	viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	
	
	viewer2->addPlane (*coefficients.get(), "plane");
	viewer.showCloud(cloud.makeShared());
	//pcl::visualization::CloudViewer viewer2("DownSampled cloud_file Viewer");
	//viewer2.showCloud(cloud2);
	while (!viewer.wasStopped()) // && !viewer2.wasStopped()
	{
		viewer2->spinOnce (100);
	// boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}


  return (0);
}