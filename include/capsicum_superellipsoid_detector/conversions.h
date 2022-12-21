#ifndef __SUPERELLIPSOID_ROS_CONVERSION_H__
#define __SUPERELLIPSOID_ROS_CONVERSION_H__

#include <capsicum_superellipsoid_detector/superellipsoid.h>
#include <superellipsoid_msgs/Superellipsoid.h>

namespace superellipsoid
{

template <typename PointT>
Superellipsoid<PointT> fromROSMsg(const superellipsoid_msgs::Superellipsoid &se_msg)
{
  Superellipsoid<PointT> se;
  (*(se.parameters_ptr))[0] = se_msg.a;
  (*(se.parameters_ptr))[1] = se_msg.b;
  (*(se.parameters_ptr))[2] = se_msg.c;
  (*(se.parameters_ptr))[3] = se_msg.e1;
  (*(se.parameters_ptr))[4] = se_msg.e2;
  (*(se.parameters_ptr))[5] = se_msg.tx;
  (*(se.parameters_ptr))[6] = se_msg.ty;
  (*(se.parameters_ptr))[7] = se_msg.tz;
  (*(se.parameters_ptr))[8] = se_msg.roll;
  (*(se.parameters_ptr))[9] = se_msg.pitch;
  (*(se.parameters_ptr))[10] = se_msg.yaw;
  return se;
}

template <typename PointT>
superellipsoid_msgs::Superellipsoid toROSMsg(const Superellipsoid<PointT> &se)
{
  superellipsoid_msgs::Superellipsoid se_msg;
  se_msg.a = (*(se.parameters_ptr))[0];
  se_msg.b = (*(se.parameters_ptr))[1];
  se_msg.c = (*(se.parameters_ptr))[2];
  se_msg.e1 = (*(se.parameters_ptr))[3];
  se_msg.e2 = (*(se.parameters_ptr))[4];
  se_msg.tx = (*(se.parameters_ptr))[5];
  se_msg.ty = (*(se.parameters_ptr))[6];
  se_msg.tz = (*(se.parameters_ptr))[7];
  se_msg.roll = (*(se.parameters_ptr))[8];
  se_msg.pitch = (*(se.parameters_ptr))[9];
  se_msg.yaw = (*(se.parameters_ptr))[10];
  se_msg.volume = se.computeVolume();
  return se_msg;
}

/*
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
*/

} // namespace superellipsoid
#endif // __SUPERELLIPSOID_ROS_CONVERSION_H__