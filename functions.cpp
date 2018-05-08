#ifdef _FUNCTIONS_H

#include "functions.h"

void computepointcloud::addCube(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, float min_x, float max_x, float min_y, float max_y, float min_z, float max_z, std::string id)
{

	PointT pt1 (min_x, min_y, min_z);
	PointT pt2 (min_x, min_y, max_z);
	PointT pt3 (max_x, min_y, max_z);
	PointT pt4 (max_x, min_y, min_z);
	PointT pt5 (min_x, max_y, min_z);
	PointT pt6 (min_x, max_y, max_z);
	PointT pt7 (max_x, max_y, max_z);
	PointT pt8 (max_x, max_y, min_z);

	viewer->addLine (pt1, pt2, 1.0, 0.0, 0.0, "1" + id );
	viewer->addLine (pt1, pt4, 1.0, 0.0, 0.0, "2" + id );
	viewer->addLine (pt1, pt5, 1.0, 0.0, 0.0, "3" + id );
	viewer->addLine (pt5, pt6, 1.0, 0.0, 0.0, "4" + id );
	viewer->addLine (pt5, pt8, 1.0, 0.0, 0.0, "5" + id );
	viewer->addLine (pt2, pt6, 1.0, 0.0, 0.0, "6" + id );
	viewer->addLine (pt6, pt7, 1.0, 0.0, 0.0, "7" + id );
	viewer->addLine (pt7, pt8, 1.0, 0.0, 0.0, "8" + id );
	viewer->addLine (pt2, pt3, 1.0, 0.0, 0.0, "9" + id );
	viewer->addLine (pt4, pt8, 1.0, 0.0, 0.0, "10" + id );
	viewer->addLine (pt3, pt4, 1.0, 0.0, 0.0, "11" + id );
	viewer->addLine (pt3, pt7, 1.0, 0.0, 0.0, "12" + id );

}

std::vector<pcl::PointCloud<PointT>::Ptr> euclideanClustering(pcl::PointCloud<PointT>::Ptr cloud,int &numCluseters)
{
	std::vector<pcl::PointIndices> cluster_indices;
	std::vector<pcl::PointCloud<PointT>::Ptr> cluster_clouds;
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (cloud);
	cluster_indices.clear();
	cluster_clouds.clear();
	pcl::EuclideanClusterExtraction<PointT> ec;

	ec.setClusterTolerance (0.02);
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (270000);
	ec.setSearchMethod (tree);

	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (cloud->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		//save the cloud at
		cluster_clouds.push_back(cloud_cluster);

			j++;
	}
	numCluseters =cluster_clouds.size();
	return cluster_clouds;
}

void computepointcloud::getBoundingBox(pcl::PointCloud< PointT >::Ptr cloud, float &min_x, float &max_x, float &min_y, float &max_y, float &min_z, float &max_z)
{
	PointT min_point_AABB;
	PointT max_point_AABB;

	pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
  feature_extractor.setInputCloud (cloud);
  feature_extractor.compute ();
  feature_extractor.getAABB (min_point_AABB, max_point_AABB);

	min_x = min_point_AABB.x;
	max_x = max_point_AABB.x;
	min_y = min_point_AABB.y;
	max_y = max_point_AABB.y;
	min_z = min_point_AABB.z;
	max_z = max_point_AABB.z;
}

pcl::PointCloud< PointT >::Ptr computepointcloud::VoxelGrid_filter(pcl::PointCloud< PointT >::Ptr cloud, float lx, float ly, float lz)
{
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (lx, ly, lz);
	sor.filter (*cloud);
	return cloud;
}

pcl::PointCloud< PointT >::Ptr computepointcloud::Filter_in_axis(pcl::PointCloud< PointT >::Ptr cloud, string axi, float min, float max, bool negative)
{
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName (axi);
	pass.setFilterLimits (min, max);
	pass.setFilterLimitsNegative (negative);
	pass.filter (*cloud);
	return cloud;
}

pcl::PointCloud< PointT >::Ptr computepointcloud::copy_pointcloud(pcl::PointCloud< PointT >::Ptr cloud)
{
	pcl::PointCloud< PointT >::Ptr copy_cloud(new pcl::PointCloud<PointT>(*cloud));
	return copy_cloud;
}


#endif
