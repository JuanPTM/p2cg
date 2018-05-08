#ifndef _FUNCTIONS_H
#define _FUNCTIONS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/time.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZRGBA PointT;
using namespace std;
namespace computepointcloud
{
  void addCube(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, float min_x, float max_x, float min_y, float max_y, float min_z, float max_z, std::string id);

  std::vector<pcl::PointCloud<PointT>::Ptr> euclideanClustering(pcl::PointCloud<PointT>::Ptr cloud,int &numCluseters);

  void getBoundingBox(pcl::PointCloud< PointT >::Ptr cloud, float &min_x, float &max_x, float &min_y, float &max_y, float &min_z, float &max_z);

  pcl::PointCloud< PointT >::Ptr Filter_in_axis(pcl::PointCloud< PointT >::Ptr cloud, string axi, float min, float max, bool negative);

  pcl::PointCloud< PointT >::Ptr VoxelGrid_filter(pcl::PointCloud< PointT >::Ptr cloud,float lx, float ly, float lz);

  pcl::PointCloud< PointT >::Ptr copy_pointcloud(pcl::PointCloud< PointT >::Ptr cloud);

}
#endif
