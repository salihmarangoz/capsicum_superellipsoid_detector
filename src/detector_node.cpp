#include <ros/ros.h>
//#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>


// globals
ros::Publisher pc_roi_pub, pc_other_pub, clusters_pub;
std::unique_ptr<tf::TransformListener> listener;


std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> seperate_roi(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl)
{
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


// copied & modified: https://pointclouds.org/documentation/region__growing_8hpp_source.html
pcl::PointCloud<pcl::PointXYZRGB>::Ptr getColoredCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_, std::vector<pcl::PointIndices> &clusters_)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

  if (!clusters_.empty ())
  {
    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();

    srand (static_cast<unsigned int> (time (nullptr)));
    std::vector<unsigned char> colors;
    for (std::size_t i_segment = 0; i_segment < clusters_.size (); i_segment++)
    {
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
    }

    colored_cloud->width = input_->width;
    colored_cloud->height = input_->height;
    colored_cloud->is_dense = input_->is_dense;
    for (const auto& i_point: *input_)
    {
      pcl::PointXYZRGB point;
      point.x = *(i_point.data);
      point.y = *(i_point.data + 1);
      point.z = *(i_point.data + 2);
      point.r = 255;
      point.g = 0;
      point.b = 0;
      colored_cloud->points.push_back (point);
    }

    int next_color = 0;
    for (const auto& i_segment : clusters_)
    {
      for (const auto& index : (i_segment.indices))
      {
        (*colored_cloud)[index].r = colors[3 * next_color];
        (*colored_cloud)[index].g = colors[3 * next_color + 1];
        (*colored_cloud)[index].b = colors[3 * next_color + 2];
      }
      next_color++;
    }
  }

  return (colored_cloud);
}


void pc_callback(const sensor_msgs::PointCloud2ConstPtr& pc_ros)
{
  ROS_INFO_ONCE("Pointcloud received...");

  // Convert sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl_raw (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*pc_ros, *pc_pcl_raw);


  // Filter NaN Values
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::shared_ptr<std::vector<int>> indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*pc_pcl_raw, *pc_pcl, *indices);


  // Transform pointcloud to world frame
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl_tf (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl_ros::transformPointCloud("world", *pc_pcl, *pc_pcl_tf, *listener);
  std_msgs::Header pc_pcl_tf_ros_header = pcl_conversions::fromPCL(pc_pcl_tf->header);
  if (pc_pcl_tf->size() == 0)
  {
    ROS_WARN("pc_pcl_tf->size() == 0");
    return;
  }


  // Seperate pointclouds to roi & non-roi using color information
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_pc, other_pc;
  std::tie(roi_pc, other_pc) = seperate_roi(pc_pcl_tf);

  if (pc_roi_pub.getNumSubscribers() > 0 )
  {
    sensor_msgs::PointCloud2::Ptr roi_pc2_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*roi_pc, *roi_pc2_ros);
    roi_pc2_ros->header = pc_pcl_tf_ros_header;
    ROS_INFO("%d", roi_pc->points.size());
    pc_roi_pub.publish(roi_pc2_ros);
  }
  if (pc_other_pub.getNumSubscribers() > 0 )
  {
    sensor_msgs::PointCloud2::Ptr other_pc2_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*other_pc, *other_pc2_ros);
    other_pc2_ros->header = pc_pcl_tf_ros_header;
    ROS_INFO("%d", other_pc->points.size());
    pc_other_pub.publish(other_pc2_ros);
  }


  // Cluster roi pointcloud
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (roi_pc);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02);
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (10000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (roi_pc);
  ec.extract (cluster_indices);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = getColoredCloud (roi_pc, cluster_indices);

  if (clusters_pub.getNumSubscribers() > 0 )
  {
    sensor_msgs::PointCloud2::Ptr debug_pc(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*colored_cloud, *debug_pc);
    debug_pc->header = pc_pcl_tf_ros_header;
    ROS_INFO("%d", colored_cloud->points.size());
    clusters_pub.publish(debug_pc);
  }


  // Extract surface normals


  // Predict roi centroids (with least squares line intersection

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "capsicum_superellipsoid_detector_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  listener = std::make_unique<tf::TransformListener>();
  pc_roi_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("pc_roi_out", 2);
  pc_other_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("pc_other_out", 2);
  clusters_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("clusters_out", 2);

  ros::Subscriber pc_sub = nh.subscribe("pc_in", 1, pc_callback);

  ros::spin();
  return 0;
}
