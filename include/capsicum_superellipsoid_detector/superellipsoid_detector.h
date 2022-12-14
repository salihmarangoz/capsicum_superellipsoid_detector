#ifndef __SUPERELLIPSOID_DETECTOR_H__
#define __SUPERELLIPSOID_DETECTOR_H__

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <capsicum_superellipsoid_detector/SuperellipsoidDetectorConfig.h>

// TODO ===========================================================
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
// TODO ===========================================================

namespace superellipsoid
{

class SuperellipsoidDetector
{
public:
    SuperellipsoidDetector(ros::NodeHandle &nh, ros::NodeHandle &priv_nh);
    ~SuperellipsoidDetector();
    void startNode();
    void startService();
    void processInput(const sensor_msgs::PointCloud2Ptr &pc2);
    void pcCallback(const sensor_msgs::PointCloud2Ptr &pc2);
    void configCallback(capsicum_superellipsoid_detector::SuperellipsoidDetectorConfig &config, uint32_t level);

    ros::NodeHandle &m_nh;
    ros::NodeHandle &m_priv_nh;
    capsicum_superellipsoid_detector::SuperellipsoidDetectorConfig m_config;
    dynamic_reconfigure::Server<capsicum_superellipsoid_detector::SuperellipsoidDetectorConfig> m_reconfigure_server;
    ros::Subscriber m_pc_sub;
    ros::Publisher m_superellipsoids_pub,
        m_clusters_pub,
        m_centers_prior_pub,
        m_centers_optimized_pub,
        m_superellipsoids_surface_pub,
        m_superellipsoids_volume_pub,
        m_superellipsoids_volume_octomap_pub,
        m_surface_normals_marker_pub,
        m_xyzlnormal_pub,
        m_missing_surfaces_pub;
    std::unique_ptr<tf::TransformListener> m_tf_listener;
};



} // namespace superellipsoid

#endif // __SUPERELLIPSOID_DETECTOR_H__