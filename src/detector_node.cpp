
#include "capsicum_superellipsoid_detector/superellipsoid.h"
#include "capsicum_superellipsoid_detector/clustering.h"

#include <chrono>
#include <ros/ros.h>
//#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <superellipsoid_msgs/Superellipsoid.h>
#include <superellipsoid_msgs/SuperellipsoidArray.h>

#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <octomap_vpp/CountingOcTree.h>
#include <octomap_vpp/NearestRegionOcTree.h>
#include <octomap_vpp/octomap_pcl.h>
#include <pcl/point_cloud.h>

// ros params
int p_min_cluster_size, p_max_cluster_size, p_max_num_iterations, p_cost_type;
double p_cluster_tolerance, p_estimate_normals_search_radius, p_estimate_cluster_center_regularization, p_pointcloud_volume_resolution, p_octree_volume_resolution, p_prior_scaling, p_prior_center;
bool p_print_ceres_summary, p_use_fibonacci_sphere_projection_sampling;
std::string p_world_frame;

// globals
ros::Publisher superellipsoids_pub,
    clusters_pub,
    centers_prior_pub,
    centers_optimized_pub,
    superellipsoids_surface_pub,
    superellipsoids_volume_pub,
    superellipsoids_volume_octomap_pub,
    surface_normals_marker_pub,
    xyzlnormal_pub;

std::unique_ptr<tf::TransformListener> listener;

void pcCallback(const sensor_msgs::PointCloud2Ptr &pc_ros)
{
  ROS_INFO("Pointcloud received...");

  /////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////// PREPROCESSING //////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////
  auto t_start_preprocessing = std::chrono::high_resolution_clock::now();

  // Convert to PCL Pointcloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*pc_ros, *pc_pcl);

  // Filter NaN Values
  std::shared_ptr<std::vector<int>> indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*pc_pcl, *pc_pcl, *indices);

  ROS_INFO("Number of data points: %lu", pc_pcl->size());

  if (pc_pcl->size() < p_min_cluster_size)
  {
    ROS_WARN("pc_pcl->size() < p_min_cluster_size");
    ROS_WARN("====== Callback finished! =======");
    return;
  }

  // Transform pointcloud to the world frame
  pcl_ros::transformPointCloud(p_world_frame, *pc_pcl, *pc_pcl, *listener);
  std_msgs::Header pc_pcl_tf_ros_header = pcl_conversions::fromPCL(pc_pcl->header);

  auto t_end_preprocessing = std::chrono::high_resolution_clock::now();
  /////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////// CLUSTERING /////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////
  auto t_start_clustering = std::chrono::high_resolution_clock::now();

  // Clustering
  std::vector<pcl::PointIndices> cluster_indices;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters = clustering::euclideanClusterExtraction(pc_pcl, cluster_indices, p_cluster_tolerance, p_min_cluster_size, p_max_cluster_size);
  //std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters = clustering::experimentalClustering(pc_pcl, cluster_indices);

  // TODO: CHECK IF FRUIT SIZE MAKES SENSE. OTHERWISE SPLIT OR DISCARD

  // Initialize superellipsoids
  std::vector<std::shared_ptr<superellipsoid::Superellipsoid<pcl::PointXYZRGB>>> superellipsoids;
  for (const auto& current_cluster_pc : clusters)
  {
    auto new_superellipsoid = std::make_shared<superellipsoid::Superellipsoid<pcl::PointXYZRGB>>(current_cluster_pc);
    new_superellipsoid->estimateNormals(p_estimate_normals_search_radius); // search_radius
    new_superellipsoid->estimateClusterCenter(p_estimate_cluster_center_regularization); // regularization
    new_superellipsoid->flipNormalsTowardsClusterCenter();
    superellipsoids.push_back(new_superellipsoid);
  }

  auto t_end_clustering = std::chrono::high_resolution_clock::now();
  /////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////// OPTIMIZATION ///////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////
  auto t_start_optimization = std::chrono::high_resolution_clock::now();

  // Optimize Superellipsoids
  std::vector<std::shared_ptr<superellipsoid::Superellipsoid<pcl::PointXYZRGB>>> converged_superellipsoids;
  for (const auto &current_superellipsoid : superellipsoids)
  {
    if ( current_superellipsoid->fit(p_print_ceres_summary, p_max_num_iterations, (superellipsoid::CostFunctionType)(p_cost_type)) )
    { // if converged
      converged_superellipsoids.push_back(current_superellipsoid);
    }
    else
    {
      ROS_WARN("Optimization failed for a cluster!");
    }
  }

  auto t_end_optimization = std::chrono::high_resolution_clock::now();
  /////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////// OUTPUT /////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////
  auto t_start_output = std::chrono::high_resolution_clock::now();

  // debug: visualize clusters
  if (clusters_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_pc = clustering::getColoredCloud(pc_pcl, cluster_indices);
    sensor_msgs::PointCloud2::Ptr debug_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*debug_pc, *debug_pc_ros);
    debug_pc_ros->header = pc_pcl_tf_ros_header;
    clusters_pub.publish(debug_pc_ros);
  }

  // debug: publish superellipsoids for converged superellipsoids
  if (superellipsoids_pub.getNumSubscribers() > 0)
  {
    superellipsoid_msgs::SuperellipsoidArray sea;
    sea.header = pc_pcl_tf_ros_header;
    for (const auto &se : converged_superellipsoids)
    {
      superellipsoid_msgs::Superellipsoid se_msg = se->generateRosMessage();
      se_msg.header = pc_pcl_tf_ros_header;
      sea.superellipsoids.push_back(se_msg);
    }
    superellipsoids_pub.publish(sea);
  }

  // debug: visualize prior cluster centers for converged superellipsoids
  if (centers_prior_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr prior_centers(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &se : converged_superellipsoids)
    {
      prior_centers->push_back(se->getEstimatedCenter());
      //ROS_INFO("%f %f %f", se->getEstimatedCenter().x, se->getEstimatedCenter().y, se->getEstimatedCenter().z);
    }
    sensor_msgs::PointCloud2::Ptr debug_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*prior_centers, *debug_pc_ros);
    debug_pc_ros->header = pc_pcl_tf_ros_header;
    centers_prior_pub.publish(debug_pc_ros);
  }

  // debug: visualize optimized centers for converged superellipsoids
  if (centers_optimized_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr optimized_centers(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &se : converged_superellipsoids)
    {
      optimized_centers->push_back(se->getOptimizedCenter());
    }
    sensor_msgs::PointCloud2::Ptr debug_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*optimized_centers, *debug_pc_ros);
    debug_pc_ros->header = pc_pcl_tf_ros_header;
    centers_optimized_pub.publish(debug_pc_ros);
  }

  // debug: visualize converged superellipsoids surface
  if (superellipsoids_surface_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr debug_pc(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &se : converged_superellipsoids)
    {
      *(debug_pc) += *(se->sampleSurface(p_use_fibonacci_sphere_projection_sampling));
    }
    sensor_msgs::PointCloud2::Ptr debug_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*debug_pc, *debug_pc_ros);
    debug_pc_ros->header = pc_pcl_tf_ros_header;
    superellipsoids_surface_pub.publish(debug_pc_ros);
  }

  // debug: visualize converged superellipsoids volume
  if (superellipsoids_volume_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr debug_pc(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &se : converged_superellipsoids)
    {
      *(debug_pc) += *(se->sampleVolume(p_pointcloud_volume_resolution));
    }
    sensor_msgs::PointCloud2::Ptr debug_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*debug_pc, *debug_pc_ros);
    debug_pc_ros->header = pc_pcl_tf_ros_header;
    superellipsoids_volume_pub.publish(debug_pc_ros);
  }

  // debug: visualize converged superellipsoids volume via octomap_vpp and include cluster idx information
  if (superellipsoids_volume_octomap_pub.getNumSubscribers() > 0)
  {
    std::shared_ptr<octomap_vpp::CountingOcTree> countingoctree(new octomap_vpp::CountingOcTree(p_octree_volume_resolution));

    int cluster_idx = 0;
    for (const auto &se : converged_superellipsoids)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr volume_pc = se->sampleVolume(p_octree_volume_resolution/3.0); // for better estimation on grids sample more dense to reduce artifacts caused by rotation

      for (const auto &p : *volume_pc)
      {
        octomap::point3d loc(p.x, p.y, p.z);
        countingoctree->setNodeCount(loc, cluster_idx);
      }

      cluster_idx++;
    }

    std::shared_ptr<octomap_msgs::Octomap> debug_octomap(new octomap_msgs::Octomap());
    //debug_octomap->header.frame_id = p_world_frame;
    //debug_octomap->header.stamp = ros::Time::now();
    debug_octomap->header = pc_pcl_tf_ros_header;
    octomap_msgs::fullMapToMsg(*countingoctree, *debug_octomap);
    superellipsoids_volume_octomap_pub.publish(*debug_octomap);
  }

  // debug: visualize surface normals via rviz markers
  if (surface_normals_marker_pub.getNumSubscribers() > 0)
  {
    // DELETE ALL MARKERS
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.ns = "surface_normals_marker";
    marker.header = pc_pcl_tf_ros_header;
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);
    surface_normals_marker_pub.publish(marker_array);
    marker_array.markers.clear();

    // PUBLISH NORMALS FOR ALL POINTS
    int id_counter=0;
    for (const auto &se : converged_superellipsoids)
    {
      auto cloud_ = se->getCloud();
      auto normals_ = se->getNormals();

      for (int i=0; i<cloud_->points.size(); i++)
      {
        visualization_msgs::Marker marker;
        //marker.header.frame_id = p_world_frame;
        //marker.header.stamp = ros::Time();
        marker.header = pc_pcl_tf_ros_header;
        marker.ns = "surface_normals_marker";
        marker.id = id_counter++;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.scale.x = 0.001; // shaft diameter
        marker.scale.y = 0.002; // head diameter
        marker.scale.z = 0.001; // head length

        geometry_msgs::Point p1, p2;
        p1.x = cloud_->points[i].x;
        p1.y = cloud_->points[i].y;
        p1.z = cloud_->points[i].z;
        p2.x = p1.x + normals_->points[i].normal_x * 0.01;
        p2.y = p1.y + normals_->points[i].normal_y * 0.01;
        p2.z = p1.z + normals_->points[i].normal_z * 0.01;

        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker_array.markers.push_back(marker);
      }
    }
    surface_normals_marker_pub.publish(marker_array);
  }

  // debug: visualize xyz, label, normals, and curvature in a single message
  if (xyzlnormal_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr debug_pc(new pcl::PointCloud<pcl::PointXYZLNormal>);

    int id_counter = 0;
    for (const auto &se : converged_superellipsoids)
    {
      auto cloud_ = se->getCloud();
      auto normals_ = se->getNormals();
      debug_pc->points.reserve(debug_pc->points.size() + cloud_->points.size());

      for (int i = 0; i < cloud_->points.size(); i++)
      {
        pcl::PointXYZLNormal p;
        p.x = cloud_->points[i].x;
        p.y = cloud_->points[i].y;
        p.z = cloud_->points[i].z;
        p.label = id_counter;
        p.normal_x = normals_->points[i].normal_x;
        p.normal_y = normals_->points[i].normal_y;
        p.normal_z = normals_->points[i].normal_z;
        p.curvature = normals_->points[i].curvature;
        debug_pc->points.push_back(p);
      }
      id_counter++;
    }

    sensor_msgs::PointCloud2::Ptr debug_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*debug_pc, *debug_pc_ros);
    debug_pc_ros->header = pc_pcl_tf_ros_header;
    xyzlnormal_pub.publish(debug_pc_ros);
  }

  auto t_end_output = std::chrono::high_resolution_clock::now();
  /////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////// LOGGING ////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////

  double elapsed_preprocessing = std::chrono::duration<double, std::milli>(t_end_preprocessing - t_start_preprocessing).count();
  double elapsed_clustering = std::chrono::duration<double, std::milli>(t_end_clustering - t_start_clustering).count();
  double elapsed_optimization = std::chrono::duration<double, std::milli>(t_end_optimization - t_start_optimization).count();
  double elapsed_output = std::chrono::duration<double, std::milli>(t_end_output - t_start_output).count();
  double elapsed_total = elapsed_preprocessing + elapsed_clustering + elapsed_optimization + elapsed_output;

  ROS_INFO("Total Elapsed Time: %f ms", elapsed_total);
  ROS_INFO("(*) Preprocessing Time: %f ms", elapsed_preprocessing);
  ROS_INFO("(*) Clustering Time: %f ms", elapsed_clustering);
  ROS_INFO("(*) Optimization Time: %f ms", elapsed_optimization);
  ROS_INFO("(*) ROS Output Computation/Publish Time: %f ms", elapsed_output);
  ROS_WARN("====== Callback finished! =======");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "capsicum_superellipsoid_detector_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  priv_nh.param("p_cost_type", p_cost_type, (int)superellipsoid::CostFunctionType::RADIAL_EUCLIDIAN_DISTANCE);
  priv_nh.param("p_min_cluster_size", p_min_cluster_size, 100);
  priv_nh.param("p_max_cluster_size", p_max_cluster_size, 10000);
  priv_nh.param("p_max_num_iterations", p_max_num_iterations, 100);
  priv_nh.param("p_cluster_tolerance", p_cluster_tolerance, 0.01);
  priv_nh.param("p_estimate_normals_search_radius", p_estimate_normals_search_radius, 0.015);
  priv_nh.param("p_estimate_cluster_center_regularization", p_estimate_cluster_center_regularization, 2.5);
  priv_nh.param("p_pointcloud_volume_resolution", p_pointcloud_volume_resolution, 0.001);
  priv_nh.param("p_octree_volume_resolution", p_octree_volume_resolution, 0.001);
  priv_nh.param("p_prior_scaling", p_prior_scaling, 0.1);
  priv_nh.param("p_prior_center", p_prior_center, 0.1);
  priv_nh.param("p_print_ceres_summary", p_print_ceres_summary, false);
  priv_nh.param("p_use_fibonacci_sphere_projection_sampling", p_use_fibonacci_sphere_projection_sampling, true);
  priv_nh.param<std::string>("p_world_frame", p_world_frame, "world");

  listener = std::make_unique<tf::TransformListener>();

  superellipsoids_pub = priv_nh.advertise<superellipsoid_msgs::SuperellipsoidArray>("superellipsoids", 2, true);
  clusters_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("clusters", 2, true);
  superellipsoids_surface_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("superellipsoids_surface", 2, true);
  centers_prior_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("centers_prior", 2, true);
  centers_optimized_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("centers_optimized", 2, true);
  superellipsoids_volume_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("superellipsoids_volume", 2, true);
  superellipsoids_volume_octomap_pub = priv_nh.advertise<octomap_msgs::Octomap>("superellipsoids_volume_octomap", 2, true);
  surface_normals_marker_pub = priv_nh.advertise<visualization_msgs::MarkerArray>("surface_normals_marker", 2, true);
  xyzlnormal_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("xyz_label_normal", 2, true);

  ros::Subscriber pc_sub = nh.subscribe("pc_in", 1, pcCallback);

  ros::spin();
  return 0;
}

