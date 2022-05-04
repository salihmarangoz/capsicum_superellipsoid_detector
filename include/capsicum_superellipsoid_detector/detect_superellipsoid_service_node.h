#pragma once
#include<ros/ros.h>
#include <superellipsoid_msgs/DetectSuperellipsoid.h>
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
    template<typename PointT>
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> separateCloudByIndices(const typename pcl::PointCloud<PointT>::ConstPtr &input_cloud, const pcl::IndicesConstPtr &indices)
    {
    typename pcl::PointCloud<PointT>::Ptr inlier_cloud(new pcl::PointCloud<PointT>), outlier_cloud(new pcl::PointCloud<PointT>);
    inlier_cloud->header = input_cloud->header;
    outlier_cloud->header = input_cloud->header;
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(input_cloud);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*inlier_cloud);
    extract.setNegative(true);
    extract.filter(*outlier_cloud);
    return std::make_pair(inlier_cloud, outlier_cloud);
    }
class SuperellipsoidDetector
{
    public:
    SuperellipsoidDetector();

    void processPointCloudCallback(const sensor_msgs::PointCloud2Ptr& pc_ros);
    bool processSuperellipsoidDetectorCallback(superellipsoid_msgs::DetectSuperellipsoid::Request& req, superellipsoid_msgs::DetectSuperellipsoid::Response& res);

    protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::Subscriber sub_pointcloud_;
    ros::ServiceServer service_se_detector_;
    std::unique_ptr<tf::TransformListener> listener_;
    std_msgs::Header pc_pcl_tf_ros_header_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl_;
    bool is_pointcloud_available_;
    ros::Publisher pub_clusters_, pub_superellipsoids_surface_;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters_;
    bool is_service_callback_active_;
    bool use_pc_callback_;
    sensor_msgs::PointCloud2::Ptr pc_surf_ros_;

    bool getPointCloud(sensor_msgs::PointCloud2& pc_ros);

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr removeActualPointsfromPrediction(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_surf_pred, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_surf_real);
};
}