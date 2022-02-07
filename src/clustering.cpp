
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



/*
    pcl::PointCloud<pcl::PointXYZ>::Ptr superellipsoids (new pcl::PointCloud<pcl::PointXYZ>);
    // fit superellipsoid
    auto parameters_ptr = fitSuperellipsoid(pc_tmp_, cp_pcl);

    if (parameters_ptr != nullptr)
    {
      roi_centers->push_back(pcl::PointXYZ(parameters_ptr->at(5), parameters_ptr->at(6), parameters_ptr->at(7))); // todo
    }
*/



/*
  pcl::PointCloud<pcl::PointXYZ>::Ptr roi_centers (new pcl::PointCloud<pcl::PointXYZ>);
  */