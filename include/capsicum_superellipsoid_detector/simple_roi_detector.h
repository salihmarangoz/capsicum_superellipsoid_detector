#pragma once

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <utility>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

namespace superellipsoid
{

class SimpleRoiDetector
{
public:
  SimpleRoiDetector(ros::NodeHandle &nhp);

  void filter(const sensor_msgs::PointCloud2ConstPtr &pc);

private:
  std::string target_frame;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pc_sub;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> transform_filter;

  ros::Publisher pc_roi_pub;
  ros::Publisher roi_only_pub;
  ros::Publisher nonroi_only_pub;

  pcl::IndicesConstPtr filterRed(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output = nullptr);
};

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> separateCloudByIndices(const typename pcl::PointCloud<PointT>::ConstPtr &input_cloud, const pcl::IndicesConstPtr &indices)
{
  typename pcl::PointCloud<PointT>::Ptr inlier_cloud(new pcl::PointCloud<PointT>), outlier_cloud(new pcl::PointCloud<PointT>);

  // temporary fix. somehow breaks without these lines
  inlier_cloud->points.reserve(input_cloud->points.size());
  outlier_cloud->points.reserve(input_cloud->points.size());

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

} // namespace superellipsoid
