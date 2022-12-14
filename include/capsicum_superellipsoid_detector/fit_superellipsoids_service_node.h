#pragma once
#include<ros/ros.h>
#include <shape_completion_bridge_msgs/FitSuperellipsoids.h>

#include <superellipsoid_msgs/SuperellipsoidArray.h>

#include "capsicum_superellipsoid_detector/superellipsoid.h"
#include "capsicum_superellipsoid_detector/clustering.h"

#include <sensor_msgs/PointCloud2.h>
#include <memory>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <pcl/filters/extract_indices.h>

namespace superellipsoid
{


class SuperellipsoidFitter
{
    public:
    SuperellipsoidFitter();

    void processPointCloudCallback(const sensor_msgs::PointCloud2Ptr& pc_ros);
    bool processSuperellipsoidFitterCallback(shape_completion_bridge_msgs::FitSuperellipsoids::Request& req, shape_completion_bridge_msgs::FitSuperellipsoids::Response& res);

    protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::ServiceServer service_se_fitter_;
    std::unique_ptr<tf::TransformListener> listener_;
    std_msgs::Header pc_pcl_tf_ros_header_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl_;
    ros::Publisher pub_clusters_, pub_superellipsoids_surface_;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters_;
    sensor_msgs::PointCloud2::Ptr pc_surf_ros_;

    int p_min_cluster_size, p_max_cluster_size, p_max_num_iterations, p_cost_type, p_missing_surfaces_num_samples;
    double p_cluster_tolerance, p_estimate_normals_search_radius, p_estimate_cluster_center_regularization, p_pointcloud_volume_resolution, p_octree_volume_resolution, p_prior_scaling, p_prior_center, p_missing_surfaces_threshold;
    bool p_print_ceres_summary, p_use_fibonacci_sphere_projection_sampling;
    std::string p_world_frame;

    bool getPointCloud(sensor_msgs::PointCloud2& pc_ros);

    void fromPCL2ROSType(shape_completion_bridge_msgs::SuperellipsoidResult& dest, pcl::PointCloud<pcl::PointXYZ>::Ptr missing_surface, pcl::PointXYZ optimised_centre, pcl::PointCloud<pcl::PointXYZ>::Ptr surface_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr volume_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, sensor_msgs::PointCloud2& observed_pointcloud)
    {
        dest.optimised_centre.x = optimised_centre.x;
        dest.optimised_centre.y = optimised_centre.y;
        dest.optimised_centre.z = optimised_centre.z;

        pcl::toROSMsg(*missing_surface, dest.missing_surface_pointcloud);    
        pcl::toROSMsg(*surface_cloud, dest.surface_pointcloud);
        pcl::toROSMsg(*volume_cloud, dest.volume_pointcloud);
        pcl::toROSMsg(*normals, dest.normals);
        dest.observed_pointcloud = observed_pointcloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr removeActualPointsfromPrediction(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_surf_pred, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_surf_real);
};
}