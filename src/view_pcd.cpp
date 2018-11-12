#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/cloud_viewer.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

int main(int argc, char** argv)
{ 
  pcl::PointCloud<pcl::PointXYZ>::Ptr incloud(new pcl::PointCloud<pcl::PointXYZ>);

  // If a pcd file name was given, use it
  if(argc > 1 and argc < 3)
  {
    cout << "Reading file" << endl;
    pcl::PCDReader reader;
    reader.read(argv[1], *incloud);
  }
  else
  {
    cout << "No filename provided";
    return -1;
  }

  pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
   
  viewer.addPointCloud(incloud);  
  while(!viewer.wasStopped()) viewer.spinOnce(); 

  return 0;
}
