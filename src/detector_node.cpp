#include <ros/ros.h>
//#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// globals
ros::Publisher pc_roi_pub, pc_other_pub;


std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> seperate_roi(sensor_msgs::PointCloud2ConstPtr pc2_ros_in)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*pc2_ros_in, *pc_pcl);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr other_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ConditionOr<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionOr<pcl::PointXYZRGB>);
  color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("h", pcl::ComparisonOps::LT, -20)));
  color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("h", pcl::ComparisonOps::GT, 35)));
  color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("s", pcl::ComparisonOps::LT, 30)));
  color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("i", pcl::ComparisonOps::LT, 30)));

  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem(true);
  condrem.setInputCloud(pc_pcl);
  condrem.setCondition(color_cond);
  condrem.filter(*other_pc);
  pcl::IndicesConstPtr indices = condrem.getRemovedIndices();

  for (auto it = std::begin(*indices); it!=std::end(*indices); ++it)
    roi_pc->push_back((*pc_pcl)[*it]);

  return std::make_tuple(roi_pc, other_pc);
}

void pc_callback(const sensor_msgs::PointCloud2ConstPtr& pc)
{
  ROS_INFO_ONCE("Pointcloud received...");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_pc, other_pc;
  std::tie(roi_pc, other_pc) = seperate_roi(pc);

  if (pc_roi_pub.getNumSubscribers() > 0 )
  {
    sensor_msgs::PointCloud2::Ptr roi_pc2_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*roi_pc, *roi_pc2_ros);
    roi_pc2_ros->header = pc->header;
    ROS_INFO("%d", roi_pc->points.size());
    pc_roi_pub.publish(roi_pc2_ros);
  }

  if (pc_other_pub.getNumSubscribers() > 0 )
  {
    sensor_msgs::PointCloud2::Ptr other_pc2_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*other_pc, *other_pc2_ros);
    other_pc2_ros->header = pc->header;
    ROS_INFO("%d", other_pc->points.size());
    pc_other_pub.publish(other_pc2_ros);
  }



}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "capsicum_superellipsoid_detector_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  ros::Subscriber pc_sub = nh.subscribe("pc_in", 1, pc_callback);
  pc_roi_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("pc_roi_out", 2);
  pc_other_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("pc_other_out", 2);

  ros::spin();
  return 0;
}
