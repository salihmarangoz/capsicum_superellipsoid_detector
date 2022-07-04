#include "capsicum_superellipsoid_detector/simple_roi_detector.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

namespace superellipsoid
{

SimpleRoiDetector::SimpleRoiDetector(ros::NodeHandle &nhp)
{
  target_frame = nhp.param<std::string>("map_frame", "world");

  tf_buffer.reset(new tf2_ros::Buffer(ros::Duration(tf2::BufferCore::DEFAULT_CACHE_TIME)));
  tf_listener.reset(new tf2_ros::TransformListener(*tf_buffer, nhp));
  pc_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nhp, "input", 1));
  transform_filter.reset(new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*pc_sub, *tf_buffer, target_frame, 1000, nhp));
  transform_filter->registerCallback(&SimpleRoiDetector::filter, this);
  roi_only_pub = nhp.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("roi_cloud", 1);
  nonroi_only_pub = nhp.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("nonroi_cloud", 1);
}

pcl::IndicesConstPtr SimpleRoiDetector::filterRed(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
  if (output == nullptr)
    output.reset(new pcl::PointCloud<pcl::PointXYZRGB>);


  pcl::ConditionOr<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionOr<pcl::PointXYZRGB>);
  color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("h", pcl::ComparisonOps::LT, -20)));
  color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("h", pcl::ComparisonOps::GT, 35)));
  color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("s", pcl::ComparisonOps::LT, 30)));
  color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("i", pcl::ComparisonOps::LT, 30)));


  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem(true);
  condrem.setInputCloud(input);
  condrem.setCondition(color_cond);
  // temporary fix. somehow breaks without these lines
  output->points.reserve(input->points.size());
  condrem.filter(*output);
  return condrem.getRemovedIndices();
}

void SimpleRoiDetector::filter(const sensor_msgs::PointCloud2ConstPtr &pc)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*pc, *pcl_cloud);


  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr not_red_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::IndicesConstPtr redIndices = filterRed(pcl_cloud);
  pcl::IndicesPtr redIndicesRo(new std::vector<int>);

  ROS_INFO_STREAM("Red indices: " << redIndices->size());

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> ro;
  ro.setInputCloud(pcl_cloud);
  ro.setIndices(redIndices);
  ro.setMeanK(5);
  ro.setStddevMulThresh(0.01);
  ro.filter(*redIndicesRo);

  geometry_msgs::TransformStamped pcFrameTf;
  try
  {
    pcFrameTf = tf_buffer->lookupTransform(target_frame, pc->header.frame_id, pc->header.stamp);
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform to map frame: " << e.what());
    return;
  }
  ROS_INFO_STREAM("Transform for time " << pc->header.stamp << " successful");

  static Eigen::Isometry3d lastTfEigen;
  Eigen::Isometry3d tfEigen = tf2::transformToEigen(pcFrameTf);

  if (tfEigen.isApprox(lastTfEigen, 1e-2)) // Publish separate clouds only if not moved
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlier_cloud, outlier_cloud;
    std::tie(inlier_cloud, outlier_cloud) = separateCloudByIndices<pcl::PointXYZRGB>(pcl_cloud, redIndicesRo);
    roi_only_pub.publish(*inlier_cloud);
    nonroi_only_pub.publish(*outlier_cloud);
  }
  lastTfEigen = tfEigen;
}

} // namespace superellipsoid



int main(int argc, char **argv)
{
  ros::init(argc, argv, "detect_roi");

  ros::NodeHandle nhp("~");

  superellipsoid::SimpleRoiDetector filter(nhp);

  ros::spin();
}
