
#ifndef CLUSTERING_H
#define CLUSTERING_H

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

namespace clustering
{

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> 
euclideanClusterExtraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, std::vector<pcl::PointIndices> &cluster_indices_out, float cluster_tolerance, int min_cluster_size, int max_cluster_size);

/*
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
experimentalClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, std::vector<pcl::PointIndices> &cluster_indices_out);

bool
experimentalClusteringCondition(const pcl::PointXYZRGBNormal& point_a, const pcl::PointXYZRGBNormal& point_b, float squared_distance);
*/

pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
getColoredCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, std::vector<pcl::PointIndices> &cluster_indices_in);

} // namespace

#endif // CLUSTERING_H