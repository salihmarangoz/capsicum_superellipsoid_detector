#include <capsicum_superellipsoid_detector/superellipsoid_detector.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace superellipsoid
{


SuperellipsoidDetector::SuperellipsoidDetector(ros::NodeHandle &nh, ros::NodeHandle &priv_nh) : m_nh(nh), m_priv_nh(priv_nh)
{
  // ROS Dynamic Reconfigure
  dynamic_reconfigure::Server<capsicum_superellipsoid_detector::SuperellipsoidDetectorConfig>::CallbackType cb;
  cb = boost::bind(&SuperellipsoidDetector::configCallback, this, _1, _2);
  m_reconfigure_server.setCallback(cb);

  // ROS TF
  // TODO: TF TO TF2
  m_tf_listener = std::make_unique<tf::TransformListener>();
}

SuperellipsoidDetector::~SuperellipsoidDetector(){}

void SuperellipsoidDetector::configCallback(capsicum_superellipsoid_detector::SuperellipsoidDetectorConfig &config, uint32_t level)
{
  ROS_INFO("config_callback");
  m_config = config;
}

void SuperellipsoidDetector::startNode()
{
  if (m_is_started)
  {
    ROS_ERROR("Node or service is already started. Only one can be started in a single process.");
  }
  m_is_started = true;

  // ROS Publishers
  m_superellipsoids_pub = m_priv_nh.advertise<superellipsoid_msgs::SuperellipsoidArray>("superellipsoids", 2, true);
  m_missing_surfaces_pub = m_priv_nh.advertise<sensor_msgs::PointCloud2>("missing_surfaces", 2, true);

  // ROS Publishers (debugging/visualization purposes)
  m_pc_preprocessed_pub = m_priv_nh.advertise<sensor_msgs::PointCloud2>("pc_preprocessed", 2, true);
  m_superellipsoids_surface_pub = m_priv_nh.advertise<sensor_msgs::PointCloud2>("superellipsoids_surface", 2, true);
  m_centers_prior_pub = m_priv_nh.advertise<sensor_msgs::PointCloud2>("centers_prior", 2, true);
  m_centers_optimized_pub = m_priv_nh.advertise<sensor_msgs::PointCloud2>("centers_optimized", 2, true);
  m_surface_normals_marker_pub = m_priv_nh.advertise<visualization_msgs::MarkerArray>("surface_normals_marker", 2, true);

  // ROS Subscribers
  m_pc_sub = m_priv_nh.subscribe("pc_in", 1, &SuperellipsoidDetector::subscriberCallback, this); // queue size: 2 for throughput, 1 for low latency
  
  // ROS Service
  m_trigger_service = m_priv_nh.advertiseService("trigger", &SuperellipsoidDetector::triggerCallback, this);
  
}

void SuperellipsoidDetector::startService()
{
  if (m_is_started)
  {
    ROS_ERROR("Node or service is already started. Only one can be started in a single process.");
  }
  m_is_started = true;

  // ROS Service
  m_fit_superellipsoids_service = m_nh.advertiseService("fit_superellipsoids", &SuperellipsoidDetector::serviceCallback, this);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////// TRIGGER CALLBACK //////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool SuperellipsoidDetector::triggerCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
{
  ROS_DEBUG("Triggered capsicum_superellipsoid_detector!");
  m_is_triggered = true;
  if (m_config.processing_mode == 0 /* CONTINUOUS */) ROS_WARN_THROTTLE(10.0, "The processing mode is CONTINUOUS. Triggering the node have no effect.");
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////// SERVICE CALLBACK //////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool SuperellipsoidDetector::serviceCallback(capsicum_superellipsoid_detector::FitSuperellipsoidsRequest& req, capsicum_superellipsoid_detector::FitSuperellipsoidsResponse& res)
{
  ROS_INFO("Request (for multiple superellipsoids) received...");

  auto t_start = std::chrono::high_resolution_clock::now();
  ROS_INFO("Pointcloud received...");

  // Config Override
  capsicum_superellipsoid_detector::SuperellipsoidDetectorConfig config = m_config;
  config.__fromMessage__(req.config_override);
  
  // Process input pointcloud
  std_msgs::Header new_header;
  superellipsoid::SuperellipsoidArray<pcl::PointXYZRGB> converged_superellipsoids;
  if ( !processInput(config, req.in_cloud, new_header, converged_superellipsoids) )
  {
    return false;
  }
  
  // Compute missing surfaces
  if (req.return_missing_surfaces)
  {
    for (auto &se : converged_superellipsoids)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr ms_pcl = se.estimateMissingSurfaces(m_config.missing_surfaces_threshold, m_config.missing_surfaces_num_samples);
        sensor_msgs::PointCloud2 tmp_pc_ros;
        pcl::toROSMsg(*ms_pcl, tmp_pc_ros);
        tmp_pc_ros.header = new_header;
        res.missing_surfaces.push_back(tmp_pc_ros);
    }
  }

  // TODO: res.outliers;

  double elapsed_time = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
  ROS_INFO("Total callback time: %f ms", elapsed_time);
  ROS_WARN("====== Callback finished! =======");
  return true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////// TOPIC CALLBACK ////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void SuperellipsoidDetector::subscriberCallback(const sensor_msgs::PointCloud2Ptr &input_pc2)
{
  // Check processing mode
  if (m_config.processing_mode != 0 /* NOT CONTINUOUS */ && !m_is_triggered) 
  {
    ROS_DEBUG("Skipped a message!");
    return;
  }
  m_is_triggered = false;

  auto t_start = std::chrono::high_resolution_clock::now();
  ROS_INFO("Pointcloud received...");
  
  // Process input pointcloud
  std_msgs::Header new_header;
  superellipsoid::SuperellipsoidArray<pcl::PointXYZRGB> converged_superellipsoids;
  processInput(m_config, *input_pc2, new_header, converged_superellipsoids); // TODO: get outliers?

  // Publish converged superellipsoids
  if (m_superellipsoids_pub.getNumSubscribers() > 0)
  {
    superellipsoid_msgs::SuperellipsoidArray sea = superellipsoid_msgs::toROSMsg(converged_superellipsoids, new_header);
    m_superellipsoids_pub.publish(sea);
  }

  // Publish superellipsoid surface
  if (m_superellipsoids_surface_pub.getNumSubscribers() > 0)
  {
    uint32_t id=0;
    pcl::PointCloud<pcl::PointXYZL>::Ptr se_surfaces_all(new pcl::PointCloud<pcl::PointXYZL>);
    for (const auto &se : converged_superellipsoids)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr sur_pcl = se.sampleSurface(m_config.use_fibonacci_sphere_projection_sampling,
                                                      true, /*apply_transformation*/
                                                      m_config.num_samples_fibonacci,
                                                      m_config.u_res_parametric, 
                                                      m_config.v_res_parametric);
      (*se_surfaces_all) += labelPointCloud<pcl::PointXYZ, pcl::PointXYZL>(*sur_pcl, id++);
    }
    sensor_msgs::PointCloud2::Ptr tmp_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*se_surfaces_all, *tmp_pc_ros);
    tmp_pc_ros->header = new_header;
    m_superellipsoids_surface_pub.publish(tmp_pc_ros);
  }

  // Publish missing surfaces
  if (m_missing_surfaces_pub.getNumSubscribers()>0)
  {
    uint32_t id=0;
    pcl::PointCloud<pcl::PointXYZL>::Ptr missing_surfaces_all(new pcl::PointCloud<pcl::PointXYZL>);
    for (auto &se : converged_superellipsoids)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr ms_pcl = se.estimateMissingSurfaces(m_config.missing_surfaces_threshold, m_config.missing_surfaces_num_samples);
      (*missing_surfaces_all) += labelPointCloud<pcl::PointXYZ, pcl::PointXYZL>(*ms_pcl, id++);
    }
    sensor_msgs::PointCloud2::Ptr tmp_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*missing_surfaces_all, *tmp_pc_ros);
    tmp_pc_ros->header = new_header;
    m_missing_surfaces_pub.publish(tmp_pc_ros);
  }

  // Publish prior centroids
  if (m_centers_prior_pub.getNumSubscribers() > 0)
  {
    uint32_t id=0;
    pcl::PointCloud<pcl::PointXYZL>::Ptr prior_centers(new pcl::PointCloud<pcl::PointXYZL>);
    for (const auto &se : converged_superellipsoids)
    {
      prior_centers->push_back( labelPoint<pcl::PointXYZ, pcl::PointXYZL>(se.getEstimatedCenter(), id++) );
    }
    sensor_msgs::PointCloud2::Ptr tmp_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*prior_centers, *tmp_pc_ros);
    tmp_pc_ros->header = new_header;
    m_centers_prior_pub.publish(tmp_pc_ros);
  }

  // Publish optimized centroids
  if (m_centers_optimized_pub.getNumSubscribers() > 0)
  {
    uint32_t id=0;
    pcl::PointCloud<pcl::PointXYZL>::Ptr optimized_centers(new pcl::PointCloud<pcl::PointXYZL>);
    for (const auto &se : converged_superellipsoids)
    {
      optimized_centers->push_back( labelPoint<pcl::PointXYZ, pcl::PointXYZL>(se.getOptimizedCenter(), id++) );
    }
    sensor_msgs::PointCloud2::Ptr tmp_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*optimized_centers, *tmp_pc_ros);
    tmp_pc_ros->header = new_header;
    m_centers_optimized_pub.publish(tmp_pc_ros);
  }

  // Publish surface normals via rviz markers
  if (m_surface_normals_marker_pub.getNumSubscribers() > 0)
  {
    // DELETE ALL MARKERS
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.ns = "surface_normals_marker";
    marker.header = new_header;
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);
    m_surface_normals_marker_pub.publish(marker_array);
    marker_array.markers.clear();

    int id_counter=0;
    for (const auto &se : converged_superellipsoids)
    {
      auto cloud_ = se.getCloud();
      auto normals_ = se.getNormals();

      for (int i=0; i<cloud_->size(); i++)
      {
        visualization_msgs::Marker marker;
        marker.header = new_header;
        marker.ns = "surface_normals_marker";
        marker.id = id_counter++;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.scale.x = 0.001; // shaft diameter
        marker.scale.y = 0.002; // head diameter
        marker.scale.z = 0.001; // head length
        marker.pose.orientation.w = 1; // fix rviz warning: Uninitialized quaternion, assuming identity.

        geometry_msgs::Point p1, p2;
        p1.x = cloud_->points[i].x;
        p1.y = cloud_->points[i].y;
        p1.z = cloud_->points[i].z;
        p2.x = p1.x + normals_->points[i].normal_x * 0.01;
        p2.y = p1.y + normals_->points[i].normal_y * 0.01;
        p2.z = p1.z + normals_->points[i].normal_z * 0.01;

        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker_array.markers.push_back(marker);
      }
    }
    m_surface_normals_marker_pub.publish(marker_array);
  }

  // Publish preprocessed pointcloud with clustering and surface normal information
  if (m_pc_preprocessed_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr tmp_pc_pcl(new pcl::PointCloud<pcl::PointXYZLNormal>);
    int id = 0;
    for (const auto &se : converged_superellipsoids)
    {
      auto cloud_ = se.getCloud();
      auto normals_ = se.getNormals();
      tmp_pc_pcl->points.reserve(tmp_pc_pcl->size() + cloud_->size());
      for (int i = 0; i < cloud_->size(); i++)
      {
        pcl::PointXYZLNormal p;
        p.x = cloud_->points[i].x;
        p.y = cloud_->points[i].y;
        p.z = cloud_->points[i].z;
        p.label = id;
        p.normal_x = normals_->points[i].normal_x;
        p.normal_y = normals_->points[i].normal_y;
        p.normal_z = normals_->points[i].normal_z;
        p.curvature = normals_->points[i].curvature;
        tmp_pc_pcl->points.push_back(p);
      }
      id++;
    }
    sensor_msgs::PointCloud2::Ptr tmp_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*tmp_pc_pcl, *tmp_pc_ros);
    tmp_pc_ros->header = new_header;
    m_pc_preprocessed_pub.publish(tmp_pc_ros);
  }
  

  double elapsed_time = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
  ROS_INFO("Total callback time: %f ms", elapsed_time);
  ROS_WARN("====== Callback finished! =======");
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool SuperellipsoidDetector::processInput(capsicum_superellipsoid_detector::SuperellipsoidDetectorConfig &config,
                                          const sensor_msgs::PointCloud2 &pc2,
                                          std_msgs::Header &new_header,
                                          std::vector<superellipsoid::Superellipsoid<pcl::PointXYZRGB>> &converged_superellipsoids
                                          )
{
  // Convert to PCL Pointcloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(pc2, *pc_pcl);

  // Filter Invalid Points
  std::shared_ptr<std::vector<int>> indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*pc_pcl, *pc_pcl, *indices);

  if (pc_pcl->size() < config.min_cluster_size)
  {
    ROS_WARN("Number of points (%lu) is smaller than min_cluster_size (%d). Aborting...", pc_pcl->size(), config.min_cluster_size);
    return false;
  }

  // Transform pointcloud to the world frame
  pcl_ros::transformPointCloud(config.world_frame, *pc_pcl, *pc_pcl, *m_tf_listener);
  new_header = pcl_conversions::fromPCL(pc_pcl->header); // <-------- use this header for output messages!
  new_header.seq = m_seq++;

  // Clustering
  std::vector<pcl::PointIndices> cluster_indices;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters = clustering::euclideanClusterExtraction(pc_pcl, cluster_indices, config.cluster_tolerance, config.min_cluster_size, config.max_cluster_size);
  
  // Clustering - Post-Processing
  // TODO: CHECK IF FRUIT SIZE MAKES SENSE. OTHERWISE SPLIT OR DISCARD

  converged_superellipsoids.reserve(clusters.size());
  int fail_counter = 0;
  for (auto& current_c : clusters)
  {
    superellipsoid::Superellipsoid<pcl::PointXYZRGB> superellipsoid(current_c);
    superellipsoid.estimateNormals(config.estimate_normals_search_radius); // search_radius
    superellipsoid.estimateClusterCenter(config.estimate_cluster_center_regularization); // regularization

    if (superellipsoid.fit(config.print_ceres_summary, config.max_num_iterations, (superellipsoid::CostFunctionType)(config.cost_type))) { 
      converged_superellipsoids.push_back(superellipsoid);
    } 
    else 
    { 
      fail_counter++;
    }
  }

  if (fail_counter > 0)
  {
    ROS_WARN("Optimization failed with %d of %lu clusters.", fail_counter, fail_counter+converged_superellipsoids.size());
  }
  if (converged_superellipsoids.size() <= 0)
  {
    return false;
  }
  return true;
}

} // namespace superellipsoid




