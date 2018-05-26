
#include "computepointcloud.h"

// void computepointcloud::addCube(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, float min_x, float max_x, float min_y, float max_y, float min_z, float max_z, std::string id)
// {
//
// 	pcl::PointXYZRGBA pt1 (min_x, min_y, min_z);
// 	pcl::PointXYZRGBA pt2 (min_x, min_y, max_z);
// 	pcl::PointXYZRGBA pt3 (max_x, min_y, max_z);
// 	pcl::PointXYZRGBA pt4 (max_x, min_y, min_z);
// 	pcl::PointXYZRGBA pt5 (min_x, max_y, min_z);
// 	pcl::PointXYZRGBA pt6 (min_x, max_y, max_z);
// 	pcl::PointXYZRGBA pt7 (max_x, max_y, max_z);
// 	pcl::PointXYZRGBA pt8 (max_x, max_y, min_z);
//
// 	viewer->addLine (pt1, pt2, 1.0, 0.0, 0.0, "1" + id );
// 	viewer->addLine (pt1, pt4, 1.0, 0.0, 0.0, "2" + id );
// 	viewer->addLine (pt1, pt5, 1.0, 0.0, 0.0, "3" + id );
// 	viewer->addLine (pt5, pt6, 1.0, 0.0, 0.0, "4" + id );
// 	viewer->addLine (pt5, pt8, 1.0, 0.0, 0.0, "5" + id );
// 	viewer->addLine (pt2, pt6, 1.0, 0.0, 0.0, "6" + id );
// 	viewer->addLine (pt6, pt7, 1.0, 0.0, 0.0, "7" + id );
// 	viewer->addLine (pt7, pt8, 1.0, 0.0, 0.0, "8" + id );
// 	viewer->addLine (pt2, pt3, 1.0, 0.0, 0.0, "9" + id );
// 	viewer->addLine (pt4, pt8, 1.0, 0.0, 0.0, "10" + id );
// 	viewer->addLine (pt3, pt4, 1.0, 0.0, 0.0, "11" + id );
// 	viewer->addLine (pt3, pt7, 1.0, 0.0, 0.0, "12" + id );
//
// }

std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> computepointcloud::euclideanClustering(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,int &numCluseters)
{
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> cluster_clouds;
	numCluseters = 0;
	if(cloud->points.size() != 0)
	{
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
		tree->setInputCloud (cloud);
		cluster_indices.clear();
		cluster_clouds.clear();
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;

		ec.setClusterTolerance (0.02);
		ec.setMinClusterSize (100);
		ec.setMaxClusterSize (270000);
		ec.setSearchMethod (tree);

		ec.setInputCloud (cloud);
		ec.extract (cluster_indices);
		int j = 0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (cloud->points[*pit]); //*
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			//save the cloud at
			cluster_clouds.push_back(cloud_cluster);

			j++;
		}
		numCluseters = cluster_clouds.size();
	}
	return cluster_clouds;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> computepointcloud::euclideanClusteringDoN(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,int &numCluseters)
{
	pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
	sor.setInputCloud (cloud_ptr);
	sor.setLeafSize (0.005f, 0.005f, 0.005f);
	sor.filter (*cloud_ptr);

	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  std::cout << "Estimating the normals" << std::endl;
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree_n (new pcl::search::KdTree<pcl::PointXYZRGBA>());
  ne.setSearchMethod (tree_n);
  ne.setRadiusSearch (0.03); // 2cm
  ne.compute (*cloud_normals);
  std::cout << "Estimated the normals" << std::endl;

  // Creating the kdtree object for the search method of the extraction
  boost::shared_ptr<pcl::KdTree<pcl::PointXYZRGBA> > tree_ec  (new pcl::KdTreeFLANN<pcl::PointXYZRGBA> ());
  tree_ec->setInputCloud (cloud);

  // Extracting Euclidean clusters using cloud and its normals
  std::vector<int> indices;
  std::vector<pcl::PointIndices> cluster_indices;
  const float tolerance = 0.01f; // 1cm tolerance in (x, y, z) coordinate system
  const double eps_angle = 40* (M_PI / 180.0); // 45degree tolerance in normals
  const unsigned int min_cluster_size = 100;

  pcl::extractEuclideanClusters (*cloud, *cloud_normals, tolerance, tree_ec, cluster_indices, eps_angle, min_cluster_size);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (cloud->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		//save the cloud at
		cluster_clouds.push_back(cloud_cluster);

		j++;
	}
	numCluseters = cluster_clouds.size();
}
return cluster_clouds;

}

void computepointcloud::getBoundingBox(pcl::PointCloud< pcl::PointXYZRGBA >::Ptr cloud, float &min_x, float &max_x, float &min_y, float &max_y, float &min_z, float &max_z)
{
	pcl::PointXYZRGBA min_point_AABB;
	pcl::PointXYZRGBA max_point_AABB;

	pcl::MomentOfInertiaEstimation <pcl::PointXYZRGBA> feature_extractor;
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

pcl::PointCloud< pcl::PointXYZRGBA >::Ptr computepointcloud::VoxelGrid_filter(pcl::PointCloud< pcl::PointXYZRGBA >::Ptr cloud, float lx, float ly, float lz)
{
	pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (lx, ly, lz);
	sor.filter (*cloud);
	return cloud;
}

pcl::PointCloud< pcl::PointXYZRGBA >::Ptr computepointcloud::Filter_in_axis(pcl::PointCloud< pcl::PointXYZRGBA >::Ptr cloud, string axi, float min, float max, bool negative)
{
	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName (axi);
	pass.setFilterLimits (min, max);
	pass.setFilterLimitsNegative (negative);
	pass.filter (*cloud);
	return cloud;
}

pcl::PointCloud< pcl::PointXYZRGBA >::Ptr computepointcloud::copy_pointcloud(pcl::PointCloud< pcl::PointXYZRGBA >::Ptr cloud)
{
	pcl::PointCloud< pcl::PointXYZRGBA >::Ptr copy_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud));
	return copy_cloud;
}

pcl::PointXYZRGBA computepointcloud::getCentroid(pcl::PointCloud< pcl::PointXYZRGBA >::Ptr cloud)
{
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid (*cloud, centroid);
	pcl::PointXYZRGBA p;
	p.x = centroid[0];
	p.y = centroid[1];
	p.z = centroid[2];
	return p;
}
