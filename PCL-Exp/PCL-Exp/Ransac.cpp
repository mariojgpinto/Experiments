#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>



#include <pcl/ModelCoefficients.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  //viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

int
main(int argc, char** argv)
{
  // initialize PointClouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_orig (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr mine (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("floor.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file floor.pcd \n");
    return (-1);
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("floor.pcd", *cloud_orig) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  } 

  
  mine.get()->push_back(pcl::PointXYZ(0,0,0));
  mine.get()->push_back(pcl::PointXYZ(1,0,0));
  mine.get()->push_back(pcl::PointXYZ(0,1,0));
  mine.get()->push_back(pcl::PointXYZ(0,0,1));
  mine.get()->push_back(pcl::PointXYZ(1,0,1));
  mine.get()->push_back(pcl::PointXYZ(0,1,1));
  mine.get()->push_back(pcl::PointXYZ(1,1,0));
  mine.get()->push_back(pcl::PointXYZ(1,1,1));

  //
  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  //pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  //// Create the segmentation object
  //pcl::SACSegmentation<pcl::PointXYZ> seg;
  //// Optional
  //seg.setOptimizeCoefficients (true);
  //// Mandatory
  //seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
  //seg.setMethodType (pcl::SAC_RANSAC);
  //seg.setDistanceThreshold (0.1);

  //seg.setInputCloud (cloud->makeShared ());
  //seg.segment (*inliers, *coefficients);


  //int as = inliers->indices.size ();
  //if (inliers->indices.size () == 0)
  //{
  //  PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  //  return (-1);
  //}

  //int i = 0;
  //for ( ; i < inliers->indices.size() ; i++){
	 // cloud->points.erase(cloud->begin() + inliers->indices.at(i));
  //}
  
  //
  //
  //
  //
  //

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.02);

  seg.setInputCloud (cloud->makeShared ());
  seg.segment (*inliers, *coefficients);

  //seg.segment (*inliers, *coefficients);

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  //std::vector<int> inliers;

  //// created RandomSampleConsensus object and compute the appropriated model

  //pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
  //  model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
  //  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
  //  ransac.setDistanceThreshold (.1);
  //  ransac.computeModel(2);
  //  ransac.getInliers(inliers);


  // copies all inliers of the model computed to another PointCloud
  //pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *final);


	


  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *final);

  pcl::ExtractIndices<pcl::PointXYZ> extract ; 
	extract.setInputCloud (cloud); 
	extract.setIndices (inliers); 
	//extract.setNegative (false); //Removes part_of_cloud but retain the original full_cloud 
	extract.setNegative (true); // Removes part_of_cloud from full cloud  and keep the rest 
	extract.filter (*cloud); 

  // creates the visualization object and adds either our orignial cloud or all of the inliers
  // depending on the command line arguments specified.
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer4;



  viewer = simpleVis(final);
  viewer2 = simpleVis(cloud);
  viewer3 = simpleVis(cloud_orig);
  viewer4 = simpleVis(mine);

  viewer2->addPlane(*coefficients);

  //if (pcl::console::find_argument (argc, argv, "-f") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
  //  viewer = simpleVis(final);
  //else
  //  viewer = simpleVis(cloud);
  while (!viewer->wasStopped () && !viewer2->wasStopped ())
  {
    viewer->spinOnce (100);
	viewer2->spinOnce (100);
	viewer3->spinOnce (100);
	viewer4->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return 0;
 }