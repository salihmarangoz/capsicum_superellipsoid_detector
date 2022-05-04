#include <capsicum_superellipsoid_detector/detect_superellipsoid_service_node.h>
#include <pcl/filters/conditional_removal.h>
 #include <pcl/kdtree/io.h>

using namespace superellipsoid;

SuperellipsoidDetector::SuperellipsoidDetector(): nhp_("~")
{
    use_pc_callback_ = false;
    sub_pointcloud_ = nh_.subscribe("pc_in", 10, &SuperellipsoidDetector::processPointCloudCallback, this);
    service_se_detector_ = nhp_.advertiseService("superellipsoid_detector_server", &SuperellipsoidDetector::processSuperellipsoidDetectorCallback, this);
    is_pointcloud_available_ = false;
    is_service_callback_active_ = false;
    pub_superellipsoids_surface_ = nhp_.advertise<sensor_msgs::PointCloud2>("superellipsoids_surface", 2, true);
    pc_surf_ros_ == NULL;
}

void SuperellipsoidDetector::processPointCloudCallback(const sensor_msgs::PointCloud2Ptr& pc_ros)
{
  ROS_INFO_ONCE("Pointcloud received...");

  /////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////// PREPROCESSING //////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////
  if(use_pc_callback_ == false)
  {
    if(pc_surf_ros_ != NULL)
    {
      pc_surf_ros_->header.stamp = ros::Time::now();
      pub_superellipsoids_surface_.publish(pc_surf_ros_);
    }
    return;
  }

  if(is_service_callback_active_)
  {
    ROS_WARN("Service callback in progress. Returning...");
  }

  // Convert to PCL Pointcloud
  pc_pcl_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*pc_ros, *pc_pcl_);

  // Filter NaN Values
  std::shared_ptr<std::vector<int>> indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*pc_pcl_, *pc_pcl_, *indices);

  if (pc_pcl_->size() == 0)
  {
    ROS_WARN("pc_pcl_tf->size() == 0");
    is_pointcloud_available_ = false;
    return;
  }
  is_pointcloud_available_ = true;
  // Transform pointcloud to the world frame
  pcl_ros::transformPointCloud("world", *pc_pcl_, *pc_pcl_, *listener_); // todo: parametric world_frame
  pc_pcl_tf_ros_header_ = pcl_conversions::fromPCL(pc_pcl_->header);


}

bool SuperellipsoidDetector::getPointCloud(sensor_msgs::PointCloud2& pc_ros)
{
  boost::shared_ptr<sensor_msgs::PointCloud2 const> temp  = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("pc_in", ros::Duration(5));
  if (temp == NULL)
  {
      ROS_INFO("No point cloud message received");
      return false;
  }
  pc_ros = *temp;
  return true;
}

pcl::PointCloud<pcl::PointXYZ>::ConstPtr SuperellipsoidDetector::removeActualPointsfromPrediction(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_surf_pred, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_surf_real)
{
  std::vector<int> indices_to_remove;
  // pcl::getApproximateIndices<pcl::PointXYZ, pcl::PointXYZRGB>(pc_surf_pred, pc_surf_real, indices_to_remove);
   
  try 
  {
    pcl::search::KdTree<pcl::PointXYZ> tree_pred; 
    tree_pred.setInputCloud (pc_surf_pred);
    float radius = 0.015f;
    for (const auto &point : pc_surf_real->points)
    {
        pcl::PointXYZ temp;
        temp.x = point.x;
        temp.y = point.y;
        temp.z = point.z;
        std::vector<int> point_indices;
        std::vector<float> point_distances;
        if(tree_pred.radiusSearch (temp, radius, point_indices, point_distances))
        {
            indices_to_remove.insert(indices_to_remove.end(), point_indices.begin(), point_indices.end());
        }
    }
    std::sort( indices_to_remove.begin(), indices_to_remove.end() );
    indices_to_remove.erase( std::unique( indices_to_remove.begin(), indices_to_remove.end() ), indices_to_remove.end() );
    ROS_WARN_STREAM("No of indices: "<<indices_to_remove.size());
    pcl::IndicesConstPtr indices_ptr(new pcl::Indices(indices_to_remove));
    const auto [inlier_cloud, outlier_cloud] = separateCloudByIndices<pcl::PointXYZ>(pc_surf_pred, indices_ptr); 
    return outlier_cloud;
  }
  catch(const std::exception &e)
  {
      ROS_WARN_STREAM("removeActualPointsfromPrediction"<<e.what());
  }
}

bool SuperellipsoidDetector::processSuperellipsoidDetectorCallback(superellipsoid_msgs::DetectSuperellipsoid::Request& req, superellipsoid_msgs::DetectSuperellipsoid::Response& res)
{
  ROS_WARN("processSuperellipsoidDetectorCallback");
  // if(is_pointcloud_available_ == false)
  // {
  //   res.success = -1;
  //   ROS_WARN("No point cloud received");
  //   return false;
  // }
  is_service_callback_active_ = true;
  sensor_msgs::PointCloud2 pc_ros;
  if(getPointCloud(pc_ros) == false)
  {
      ROS_WARN("Returning from service call");
      res.success = -1;
      return false;
  }
  // Convert to PCL Pointcloud
  pc_pcl_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(pc_ros, *pc_pcl_);

  // Filter NaN Values
  std::shared_ptr<std::vector<int>> indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*pc_pcl_, *pc_pcl_, *indices);

  if (pc_pcl_->size() == 0)
  {
    ROS_WARN("pc_pcl_tf->size() == 0");
    is_pointcloud_available_ = false;
    return false;
  }
  is_pointcloud_available_ = true;
  // Transform pointcloud to the world frame
  pcl_ros::transformPointCloud("world", *pc_pcl_, *pc_pcl_, *listener_); // todo: parametric world_frame
  pc_pcl_tf_ros_header_ = pcl_conversions::fromPCL(pc_pcl_->header);

     // Clustering
  std::vector<pcl::PointIndices> cluster_indices;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters = clustering::euclideanClusterExtraction(pc_pcl_, cluster_indices, 0.010, 100, 10000);
  //std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters = clustering::experimentalClustering(pc_pcl, cluster_indices);

  // (TODO) HERE <-- CHECK IF FRUIT SIZE MAKES SENSE. OTHERWISE SPLIT OR DISCARD

  // debug: visualize clusters
  if (pub_clusters_.getNumSubscribers() > -1)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_pc = clustering::getColoredCloud(pc_pcl_, cluster_indices);
    sensor_msgs::PointCloud2::Ptr debug_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*debug_pc, *debug_pc_ros);
    debug_pc_ros->header = pc_pcl_tf_ros_header_;
    pub_clusters_.publish(debug_pc_ros);
  }

  // Initialize superellipsoids
  std::vector<std::shared_ptr<superellipsoid::Superellipsoid<pcl::PointXYZRGB>>> superellipsoids;
  for (const auto& current_cluster_pc : clusters)
  {
    auto new_superellipsoid = std::make_shared<superellipsoid::Superellipsoid<pcl::PointXYZRGB>>(current_cluster_pc);
    new_superellipsoid->estimateNormals(0.015); // search_radius
    new_superellipsoid->estimateClusterCenter(2.5); // regularization
    new_superellipsoid->flipNormalsTowardsClusterCenter();
    superellipsoids.push_back(new_superellipsoid);
  }
  // Optimize Superellipsoids
  std::vector<std::shared_ptr<superellipsoid::Superellipsoid<pcl::PointXYZRGB>>> converged_superellipsoids;
  for (const auto &current_superellipsoid : superellipsoids)
  {
    if ( current_superellipsoid->fit(true) ) // print ceres summary
    { // if converged
      converged_superellipsoids.push_back(current_superellipsoid);
    }
  }

  // debug: publish superellipsoids for converged superellipsoids

  superellipsoid_msgs::SuperellipsoidArray sea;
  sea.header = pc_pcl_tf_ros_header_;
  for (const auto &se : converged_superellipsoids)
  {
    superellipsoid_msgs::Superellipsoid se_msg = se->generateRosMessage();
    se_msg.header = pc_pcl_tf_ros_header_;
    sea.superellipsoids.push_back(se_msg);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_surf(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto &se : converged_superellipsoids)
  {
    *(pc_surf) += *(se->sampleSurface());
  }
  sensor_msgs::PointCloud2::Ptr pc_surf_ros(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*pc_surf, *pc_surf_ros);
  pc_surf_ros->header = pc_pcl_tf_ros_header_;

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc_surf_pred = removeActualPointsfromPrediction(pc_surf, pc_pcl_);

  sensor_msgs::PointCloud2::Ptr pc_surf_pred_ros(new sensor_msgs::PointCloud2);

  pcl::toROSMsg(*pc_surf_pred, *pc_surf_pred_ros);
  pc_surf_pred_ros->header = pc_pcl_tf_ros_header_;

  pc_surf_ros_.reset(new sensor_msgs::PointCloud2(*pc_surf_pred_ros));

  res.superellipsoid_surface = *pc_surf_ros;
  res.superellipsoid_surface_missing_surface = *pc_surf_pred_ros; 
  res.success = 0;
  is_service_callback_active_ = false;

  ROS_WARN("**********************Service Call Finished");

  return true;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "superellipsoid_detector_service");
  ros::NodeHandle n;
  SuperellipsoidDetector se;
  ROS_INFO("Ready to Detect Superellipsoids");
  ros::spin();
  return 0;
}