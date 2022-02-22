
#include "capsicum_superellipsoid_detector/superellipsoid.h"
#include "capsicum_superellipsoid_detector/clustering.h"

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

#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <octomap_vpp/CountingOcTree.h>
#include <octomap_vpp/NearestRegionOcTree.h>
#include <octomap_vpp/octomap_pcl.h>
#include <pcl/point_cloud.h>

// globals
ros::Publisher clusters_pub,
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
  ROS_INFO_ONCE("Pointcloud received...");

  /////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////// PREPROCESSING //////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////

  // Convert to PCL Pointcloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*pc_ros, *pc_pcl);

  // Filter NaN Values
  std::shared_ptr<std::vector<int>> indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*pc_pcl, *pc_pcl, *indices);

  // Transform pointcloud to the world frame
  pcl_ros::transformPointCloud("world", *pc_pcl, *pc_pcl, *listener); // todo: parametric world_frame
  std_msgs::Header pc_pcl_tf_ros_header = pcl_conversions::fromPCL(pc_pcl->header);
  if (pc_pcl->size() == 0)
  {
    ROS_WARN("pc_pcl_tf->size() == 0");
    return;
  }

  /////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////// CLUSTERING /////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////

  // Clustering
  std::vector<pcl::PointIndices> cluster_indices;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters = clustering::euclideanClusterExtraction(pc_pcl, cluster_indices, 0.015, 100, 10000);
  //std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters = clustering::experimentalClustering(pc_pcl, cluster_indices);

  // (TODO) HERE <-- CHECK IF FRUIT SIZE MAKES SENSE. OTHERWISE SPLIT OR DISCARD

  // debug: visualize clusters
  if (clusters_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_pc = clustering::getColoredCloud(pc_pcl, cluster_indices);
    sensor_msgs::PointCloud2::Ptr debug_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*debug_pc, *debug_pc_ros);
    debug_pc_ros->header = pc_pcl_tf_ros_header;
    clusters_pub.publish(debug_pc_ros);
  }

  // Initialize superellipsoids
  std::vector<std::shared_ptr<superellipsoid::Superellipsoid>> superellipsoids;
  for (const auto& current_cluster_pc : clusters)
  {
    auto new_superellipsoid = std::make_shared<superellipsoid::Superellipsoid>(current_cluster_pc);
    new_superellipsoid->estimateNormals(0.015); // search_radius
    new_superellipsoid->estimateClusterCenter(2.5); // regularization
    new_superellipsoid->estimateNormals(0.015);      // CALL AGAIN TO COMPUTE NORMALS W.R.T. ESTIMATED CLUSTER CENTER VIEWPOINT
    superellipsoids.push_back(new_superellipsoid);
  }

  /////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////// OPTIMIZATION ///////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////

  // Optimize Superellipsoids
  std::vector<std::shared_ptr<superellipsoid::Superellipsoid>> converged_superellipsoids;
  for (const auto &current_superellipsoid : superellipsoids)
  {
    if ( current_superellipsoid->fit(true) ) // print ceres summary
    { // if converged
      converged_superellipsoids.push_back(current_superellipsoid);
    }
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
      *(debug_pc) += *(se->sampleSurface());
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
      *(debug_pc) += *(se->sampleVolume(0.001)); // todo
    }
    sensor_msgs::PointCloud2::Ptr debug_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*debug_pc, *debug_pc_ros);
    debug_pc_ros->header = pc_pcl_tf_ros_header;
    superellipsoids_volume_pub.publish(debug_pc_ros);
  }

  // debug: visualize converged superellipsoids volume via octomap_vpp and include cluster idx information
  if (superellipsoids_volume_octomap_pub.getNumSubscribers() > 0)
  {
    std::shared_ptr<octomap_vpp::CountingOcTree> countingoctree(new octomap_vpp::CountingOcTree(0.005)); // todo

    int cluster_idx = 0;
    for (const auto &se : converged_superellipsoids)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr volume_pc = se->sampleVolume(0.001); // todo

      for (const auto &p : *volume_pc)
      {
        octomap::point3d loc(p.x, p.y, p.z);
        countingoctree->setNodeCount(loc, cluster_idx);
      }

      cluster_idx++;
    }

    std::shared_ptr<octomap_msgs::Octomap> debug_octomap(new octomap_msgs::Octomap());
    //debug_octomap->header.frame_id = "world";
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
        //marker.header.frame_id = "world";
        //marker.header.stamp = ros::Time();
        marker.header = pc_pcl_tf_ros_header;
        marker.ns = "surface_normals_marker";
        marker.id = id_counter++;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.a = 1.0; // Don't forget to set the alpha!
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

  

  ROS_WARN("Callback finished!");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "capsicum_superellipsoid_detector_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  listener = std::make_unique<tf::TransformListener>();

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

