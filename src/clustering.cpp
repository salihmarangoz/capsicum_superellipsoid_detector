
#include "capsicum_superellipsoid_detector/clustering.h"

namespace clustering
{
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> 
euclideanClusterExtraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, std::vector<pcl::PointIndices> &cluster_indices_out, float cluster_tolerance, int min_cluster_size, int max_cluster_size)
{
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_in);
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (cluster_tolerance); // 0.02
  ec.setMinClusterSize (min_cluster_size); // 50
  ec.setMaxClusterSize (max_cluster_size); // 10000
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_in);
  cluster_indices_out.reserve(64);
  ec.extract (cluster_indices_out);

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> all_clusters;
  all_clusters.reserve(cluster_indices_out.size());

  for (const auto& i_segment : cluster_indices_out)
  {
    // Gather points for a segment
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    current_cluster->reserve(i_segment.indices.size());
    for (const auto& i_point : (i_segment.indices))
    {
      current_cluster->push_back(cloud_in->at(i_point));
    }
    all_clusters.push_back(current_cluster);
  }

  return all_clusters;
}

/*
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
experimentalClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, std::vector<pcl::PointIndices> &cluster_indices_out)
{
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_in);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::copyPointCloud (*cloud_in, *cloud_with_normals);

  // Compute normals
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
  ne.setInputCloud (cloud_in);
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.03);
  ne.compute(*cloud_with_normals);

  // Set up a Conditional Euclidean Clustering class
  pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBNormal> cec (true);
  cec.setInputCloud (cloud_with_normals);
  cec.setConditionFunction (&experimentalClusteringCondition);
  cec.setClusterTolerance (0.02);
  cec.setMinClusterSize (100);
  cec.setMaxClusterSize (10000);
  cec.segment (cluster_indices_out);
  //cec.getRemovedClusters (small_clusters, large_clusters);

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> all_clusters;
  all_clusters.reserve(cluster_indices_out.size());

  for (const auto& i_segment : cluster_indices_out)
  {
    // Gather points for a segment
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    current_cluster->reserve(i_segment.indices.size());
    for (const auto& i_point : (i_segment.indices))
    {
      current_cluster->push_back(cloud_in->at(i_point));
    }
    all_clusters.push_back(current_cluster);
  }

  return all_clusters;
}

bool
experimentalClusteringCondition(const pcl::PointXYZRGBNormal& point_a, const pcl::PointXYZRGBNormal& point_b, float squared_distance)
{
  //Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap(), point_b_normal = point_b.getNormalVector3fMap();
  //printf("%f ", std::abs(point_a_normal.dot(point_b_normal)));
  //if (std::abs(point_a_normal.dot(point_b_normal)) > 0.9)
  //{
  //  return true;
  /}
  //return false;

  Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap(), point_b_normal = point_b.getNormalVector3fMap();
  Eigen::Vector3f d = point_a.getVector3fMap() - point_b.getVector3fMap();
  d.normalize();

  printf("%f ", std::abs(d.dot(point_a_normal)) + std::abs(d.dot(point_b_normal)));
  if (std::abs(d.dot(point_a_normal)) + std::abs(d.dot(point_b_normal)) < 0.1)
  {
    return true;
  }
  return false;
}
*/


// copied & modified from https://pointclouds.org/documentation/region__growing_8hpp_source.html
pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
getColoredCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, std::vector<pcl::PointIndices> &cluster_indices_in)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

  if (!cluster_indices_in.empty ())
  {
    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();

    srand (static_cast<unsigned int> (time (nullptr)));
    std::vector<unsigned char> colors;
    colors.reserve(cluster_indices_in.size()*3);

    for (std::size_t i_segment = 0; i_segment < cluster_indices_in.size (); i_segment++)
    {
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
    }

    colored_cloud->points.reserve(cloud_in->points.size());
    colored_cloud->width = cloud_in->width;
    colored_cloud->height = cloud_in->height;
    colored_cloud->is_dense = cloud_in->is_dense;

    for (const auto& i_point: *cloud_in)
    {
      pcl::PointXYZRGB point;
      point.x = *(i_point.data);
      point.y = *(i_point.data + 1);
      point.z = *(i_point.data + 2);
      point.r = 255;
      point.g = 0;
      point.b = 0;
      colored_cloud->points.push_back (point);
    }

    int next_color = 0;
    for (const auto& i_segment : cluster_indices_in)
    {
      for (const auto& index : (i_segment.indices))
      {
        (*colored_cloud)[index].r = colors[3 * next_color];
        (*colored_cloud)[index].g = colors[3 * next_color + 1];
        (*colored_cloud)[index].b = colors[3 * next_color + 2];
      }
      next_color++;
    }
  }

  return (colored_cloud);
}



} // namespace
