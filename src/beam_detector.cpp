#include <iostream>
#include <string>
#include <stdlib.h>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <pcl/console/parse.h> 
#include <pcl/common/common_headers.h> 
#include <pcl/common/angles.h> 
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/sample_consensus/sac_model_normal_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>



int main (int argc, char** argv)
{

  std::string user_name(std::getenv("USER"));   //get computer user name for file path below

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inv(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);

  //load point cloud
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/"+user_name+"/Beam-Detector/src/PCD/uneven_bed_2.pcd", *cloud_in) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file beam_1.pcd \n");
    return (-1);

  }

  //make copy of cloud to manipulate
  pcl::copyPointCloud(*cloud_in, *cloud);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

  std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;


  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  //Find ground plane

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  //Create extract object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointCloud_array;

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  // Extract the inliers
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*extracted_cloud);

  //store cloud output into array
  pointCloud_array.push_back(extracted_cloud);

  //Create the filtering object
  extract.setNegative (true);
  extract.filter (*cloud_inv);
  cloud.swap (cloud_inv);

    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  viewer->addPlane(*coefficients,"plane0");

  //Find plane perpendicular to ground plane
  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setAxis (Eigen::Vector3f (coefficients->values[0], coefficients->values[1], coefficients->values[2]));
  seg.setDistanceThreshold (0.05);
  seg.setEpsAngle (pcl::deg2rad (20.));


  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  }

  // Extract the inliers
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*extracted_cloud);

  //store cloud output into array
  pointCloud_array.push_back(extracted_cloud);

  std::cerr << "Point cloud array size: " << pointCloud_array.size() << std::endl;

  //View pointcloud
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud_in, "Cloud_in");
  viewer->addPlane(*coefficients,"plane1");

  for(int i = 0; i < pointCloud_array.size(); ++i)
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colour_rgb (pointCloud_array[i], (rand() % 255), (rand() % 255), 255); 
    viewer->addPointCloud<pcl::PointXYZ> (pointCloud_array[i], colour_rgb, "plane " + std::to_string(i));
  }

  viewer->addCoordinateSystem (0.5);
  viewer->initCameraParameters ();

  //display 
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return (0);
}

