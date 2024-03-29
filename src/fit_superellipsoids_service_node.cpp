#include <capsicum_superellipsoid_detector/fit_superellipsoids_service_node.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/io.h>

using namespace superellipsoid;

SuperellipsoidFitter::SuperellipsoidFitter(): nhp_("~")
{
    service_se_fitter_ = nhp_.advertiseService("fit_superellipsoids", &SuperellipsoidFitter::processSuperellipsoidFitterCallback, this);
    pub_superellipsoids_surface_ = nhp_.advertise<sensor_msgs::PointCloud2>("superellipsoids_surface", 2, true);
    nhp_.param("p_cost_type", p_cost_type, (int)superellipsoid::CostFunctionType::RADIAL_EUCLIDIAN_DISTANCE);
    nhp_.param("p_min_cluster_size", p_min_cluster_size, 100);
    nhp_.param("p_max_cluster_size", p_max_cluster_size, 10000);
    nhp_.param("p_max_num_iterations", p_max_num_iterations, 100);
    nhp_.param("p_missing_surfaces_num_samples", p_missing_surfaces_num_samples, 500);
    nhp_.param("p_missing_surfaces_threshold", p_missing_surfaces_threshold, 0.015);
    nhp_.param("p_cluster_tolerance", p_cluster_tolerance, 0.01);
    nhp_.param("p_estimate_normals_search_radius", p_estimate_normals_search_radius, 0.015);
    nhp_.param("p_estimate_cluster_center_regularization", p_estimate_cluster_center_regularization, 2.5);
    nhp_.param("p_pointcloud_volume_resolution", p_pointcloud_volume_resolution, 0.001);
    nhp_.param("p_octree_volume_resolution", p_octree_volume_resolution, 0.001);
    nhp_.param("p_prior_scaling", p_prior_scaling, 0.1);
    nhp_.param("p_prior_center", p_prior_center, 0.1);
    nhp_.param("p_print_ceres_summary", p_print_ceres_summary, false);
    nhp_.param("p_use_fibonacci_sphere_projection_sampling", p_use_fibonacci_sphere_projection_sampling, true);
    nhp_.param<std::string>("p_world_frame", p_world_frame, "world");
    pc_surf_ros_ == NULL;
}

pcl::PointCloud<pcl::PointXYZ>::ConstPtr SuperellipsoidFitter::removeActualPointsfromPrediction(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_surf_pred, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_surf_real)
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
    ROS_INFO_STREAM("No of indices: "<<indices_to_remove.size());
    pcl::IndicesConstPtr indices_ptr(new pcl::Indices(indices_to_remove));
    const auto [inlier_cloud, outlier_cloud] = separateCloudByIndices<pcl::PointXYZ>(pc_surf_pred, indices_ptr); 
    return outlier_cloud;
  }
  catch(const std::exception &e)
  {
      ROS_WARN_STREAM("removeActualPointsfromPrediction"<<e.what());
  }
}

bool SuperellipsoidFitter::processSuperellipsoidFitterCallback(shape_completion_bridge_msgs::FitSuperellipsoids::Request& req, shape_completion_bridge_msgs::FitSuperellipsoids::Response& res)
{
  ROS_INFO("processSuperellipsoidFitterCallback");
  int i = 0; 
  std::vector<std::shared_ptr<superellipsoid::Superellipsoid<pcl::PointXYZRGB>>> superellipsoids;
  ROS_ERROR_STREAM("No of clusters: "<<req.clustered_shapes.size());
  for (const auto& current_cluster_shape : req.clustered_shapes)
  {
    i++; 
    ROS_INFO_STREAM("***********Cluster "<<i<<" processing started***********************");
    shape_completion_bridge_msgs::SuperellipsoidResult se_res;
    ROS_DEBUG("Before cluster assignment");
    auto current_cluster_pc_ros = current_cluster_shape.cluster_pointcloud;
    ROS_DEBUG("After cluster assignment");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
    ROS_DEBUG("Before fromROSMsg");

    pcl::fromROSMsg(current_cluster_pc_ros, *pc_pcl);
    auto new_superellipsoid = std::make_shared<superellipsoid::Superellipsoid<pcl::PointXYZRGB>>(pc_pcl);
    ROS_DEBUG("Before estimateNormals");
    new_superellipsoid->estimateNormals(p_estimate_normals_search_radius); // search_radius
    ROS_DEBUG("Before estimateClusterCenter");
    new_superellipsoid->estimateClusterCenter(p_estimate_cluster_center_regularization); // regularization
    ROS_DEBUG("Before flipNormalsTowardsClusterCenter");
    new_superellipsoid->flipNormalsTowardsClusterCenter();
    superellipsoids.push_back(new_superellipsoid);

    ROS_DEBUG("\t***********Before fit***********************");
    bool success = new_superellipsoid->fit(p_print_ceres_summary, p_max_num_iterations, (superellipsoid::CostFunctionType)(p_cost_type));
    ROS_INFO_STREAM("\t***********Cluster Fitting Result "<<success<<"***********************");
    if (success)
    { // if converged
      se_res.valid_completion = true;
      ROS_DEBUG("\t\tBefore estimateMissingSurfaces");
      pcl::PointCloud<pcl::PointXYZ>::Ptr missing_surface = new_superellipsoid->estimateMissingSurfaces(p_missing_surfaces_threshold, p_missing_surfaces_num_samples);
      ROS_DEBUG("\t\tBefore getOptimizedCenter");
      pcl::PointXYZ optimised_centre = new_superellipsoid->getOptimizedCenter();
      ROS_DEBUG("\t\tBefore sampleSurface");
      pcl::PointCloud<pcl::PointXYZ>::Ptr surface_cloud = new_superellipsoid->sampleSurface(p_use_fibonacci_sphere_projection_sampling);
      ROS_DEBUG("\t\tBefore sampleVolume");
      pcl::PointCloud<pcl::PointXYZ>::Ptr volume_cloud  = new_superellipsoid->sampleVolume(p_pointcloud_volume_resolution);
      ROS_DEBUG("\t\tBefore getNormals");
      pcl::PointCloud<pcl::Normal>::Ptr normals    = new_superellipsoid->getNormals();
      ROS_DEBUG("\t\tBefore fromPCL2ROSType");
      fromPCL2ROSType(se_res, missing_surface, optimised_centre, surface_cloud, volume_cloud, normals, current_cluster_pc_ros);
      ROS_DEBUG("\tAfter fromPCL2ROSType");
      res.superellipsoids.push_back(se_res);
      ROS_DEBUG("\t\tAfter push_back");
    }
    else
    {
      ROS_WARN_STREAM("Optimization failed for cluster "<<i<<" !");
      se_res.valid_completion = false;
    }
    ROS_DEBUG_STREAM("***********Cluster "<<i<<" processing ends***********************");
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "superellipsoid_fitter_service");
  ros::NodeHandle n;
  SuperellipsoidFitter se;
  ROS_INFO("Ready to Fit Superellipsoids");
  ros::spin();
  return 0;
}