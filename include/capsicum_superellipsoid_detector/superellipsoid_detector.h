#ifndef __SUPERELLIPSOID_DETECTOR_H__
#define __SUPERELLIPSOID_DETECTOR_H__

#include <chrono>

#include <ros/ros.h>
#include <tf/transform_listener.h>

// reconfigure
#include <dynamic_reconfigure/server.h>
#include <capsicum_superellipsoid_detector/SuperellipsoidDetectorConfig.h>

// services
#include <capsicum_superellipsoid_detector/FitSuperellipsoids.h>
#include <std_srvs/Empty.h>

// topics
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

// superellipsoid_msgs
#include <superellipsoid_msgs/Superellipsoid.h>
#include <superellipsoid_msgs/SuperellipsoidArray.h>

// capsicum_superellipsoid_detector
#include "capsicum_superellipsoid_detector/superellipsoid.h"
#include "capsicum_superellipsoid_detector/clustering.h"
#include "capsicum_superellipsoid_detector/conversions.h"
#include "capsicum_superellipsoid_detector/utils.h"

namespace superellipsoid
{

class SuperellipsoidDetector
{
public:
  SuperellipsoidDetector(ros::NodeHandle &nh, ros::NodeHandle &priv_nh);
  ~SuperellipsoidDetector();
  void startNode();
  void startService();
  bool processInput(capsicum_superellipsoid_detector::SuperellipsoidDetectorConfig &config,
                                        const sensor_msgs::PointCloud2 &pc2,
                                        std_msgs::Header &new_header,
                                        std::vector<superellipsoid::Superellipsoid<pcl::PointXYZRGB>> &converged_superellipsoids);
                      
  void subscriberCallback(const sensor_msgs::PointCloud2Ptr &pc2);
  bool serviceCallback(capsicum_superellipsoid_detector::FitSuperellipsoidsRequest& req, capsicum_superellipsoid_detector::FitSuperellipsoidsResponse& res);
  void configCallback(capsicum_superellipsoid_detector::SuperellipsoidDetectorConfig &config, uint32_t level);
  bool triggerCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);

  bool m_is_started = false;
  bool m_is_triggered = false;
  uint32_t m_seq = 0;

  ros::NodeHandle &m_nh;
  ros::NodeHandle &m_priv_nh;
  capsicum_superellipsoid_detector::SuperellipsoidDetectorConfig m_config;
  dynamic_reconfigure::Server<capsicum_superellipsoid_detector::SuperellipsoidDetectorConfig> m_reconfigure_server;
  ros::Subscriber m_pc_sub;
  ros::Publisher m_superellipsoids_pub,
                  m_centers_prior_pub,
                  m_centers_optimized_pub,
                  m_superellipsoids_surface_pub,
                  m_surface_normals_marker_pub,
                  m_pc_preprocessed_pub,
                  m_missing_surfaces_pub;
  std::unique_ptr<tf::TransformListener> m_tf_listener;
  ros::ServiceServer m_fit_superellipsoids_service;
  ros::ServiceServer m_trigger_service;
};


} // namespace superellipsoid

#endif // __SUPERELLIPSOID_DETECTOR_H__