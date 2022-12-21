#include <capsicum_superellipsoid_detector/superellipsoid_detector.h>

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
  if (is_started)
  {
    ROS_ERROR("Node or service is already started. Only one can be started in a single process.");
  }
  is_started = true;

  // ROS Publishers
  m_superellipsoids_pub = m_priv_nh.advertise<superellipsoid_msgs::SuperellipsoidArray>("superellipsoids", 2, true);
  m_xyzlnormal_pub = m_priv_nh.advertise<sensor_msgs::PointCloud2>("xyz_label_normal", 2, true);
  m_missing_surfaces_pub = m_priv_nh.advertise<sensor_msgs::PointCloud2>("missing_surfaces", 2, true);

  // ROS Publishers (debugging/visualization purposes)
  m_superellipsoids_surface_pub = m_priv_nh.advertise<sensor_msgs::PointCloud2>("superellipsoids_surface", 2, true);
  m_centers_prior_pub = m_priv_nh.advertise<sensor_msgs::PointCloud2>("centers_prior", 2, true);
  m_centers_optimized_pub = m_priv_nh.advertise<sensor_msgs::PointCloud2>("centers_optimized", 2, true);
  m_surface_normals_marker_pub = m_priv_nh.advertise<visualization_msgs::MarkerArray>("surface_normals_marker", 2, true);
  m_superellipsoids_volume_pub = m_priv_nh.advertise<sensor_msgs::PointCloud2>("superellipsoids_volume", 2, true);

  // ROS Subscribers
  m_pc_sub = m_nh.subscribe("pc_in", 1, &SuperellipsoidDetector::pcCallback, this); // queue size: 2 for throughput, 1 for low latency
}

void SuperellipsoidDetector::startService()
{
  if (is_started)
  {
    ROS_ERROR("Node or service is already started. Only one can be started in a single process.");
  }
  is_started = true;


}

void SuperellipsoidDetector::pcCallback(const sensor_msgs::PointCloud2Ptr &pc2)
{
  processInput(pc2, m_config)
}

void SuperellipsoidDetector::processInput(const sensor_msgs::PointCloud2Ptr &pc2,
                                          capsicum_superellipsoid_detector::SuperellipsoidDetectorConfig &config,
                                          std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> missing_surfaces)
{
  double elapsed_total = 0;
  ROS_INFO("Pointcloud received...");

  // ----------------------------------------------------------------------------------------
  auto t_start_preprocessing = std::chrono::high_resolution_clock::now();

  // Convert to PCL Pointcloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*pc2, *pc_pcl);

  // Filter Invalid Points
  std::shared_ptr<std::vector<int>> indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*pc_pcl, *pc_pcl, *indices);

  if (pc_pcl->size() < m_config.min_cluster_size)
  {
    ROS_WARN("Number of points (%lu) is smaller than min_cluster_size (%d). Aborting...", pc_pcl->size(), m_config.min_cluster_size);
    return;
  }

  // Transform pointcloud to the world frame
  pcl_ros::transformPointCloud(m_config.world_frame, *pc_pcl, *pc_pcl, *m_tf_listener);
  std_msgs::Header pc_pcl_tf_ros_header = pcl_conversions::fromPCL(pc_pcl->header);

  double elapsed_preprocessing = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start_preprocessing).count();
  elapsed_total += elapsed_preprocessing;
  // ----------------------------------------------------------------------------------------
  auto t_start_clustering = std::chrono::high_resolution_clock::now();

  std::vector<pcl::PointIndices> cluster_indices;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters = clustering::euclideanClusterExtraction(pc_pcl, cluster_indices, m_config.cluster_tolerance, m_config.min_cluster_size, m_config.max_cluster_size);
  // TODO: CHECK IF FRUIT SIZE MAKES SENSE. OTHERWISE SPLIT OR DISCARD

  // Initialize superellipsoids
  std::vector<std::shared_ptr<superellipsoid::Superellipsoid<pcl::PointXYZRGB>>> found_superellipsoids;
  for (const auto& current_c : clusters)
  {
    found_superellipsoids.push_back( std::make_shared<superellipsoid::Superellipsoid<pcl::PointXYZRGB>>(current_c) );
  }

  double elapsed_clustering = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start_clustering).count();
  elapsed_total+=elapsed_clustering;
  // ----------------------------------------------------------------------------------------
  auto t_start_optimization = std::chrono::high_resolution_clock::now();

  // Estimate surface normals and use it for estimating fruit centers
  for (const auto& current_se : found_superellipsoids)
  {
    current_se->estimateNormals(m_config.estimate_normals_search_radius); // search_radius
    current_se->estimateClusterCenter(m_config.estimate_cluster_center_regularization); // regularization
  }

  // Optimize Superellipsoids
  std::vector<std::shared_ptr<superellipsoid::Superellipsoid<pcl::PointXYZRGB>>> converged_superellipsoids;
  int fail_counter = 0;
  for (const auto &current_superellipsoid : found_superellipsoids)
  {
    if ( current_superellipsoid->fit(m_config.print_ceres_summary, m_config.max_num_iterations, (superellipsoid::CostFunctionType)(m_config.cost_type)) )
    {
      converged_superellipsoids.push_back(current_superellipsoid);
    }
    else
    {
      fail_counter++;
    }
  }

  if (fail_counter > 0)
  {
    ROS_WARN("Optimization failed with %d of %lu clusters.", fail_counter, converged_superellipsoids.size());
  }

  double elapsed_optimization = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start_optimization).count();
  elapsed_total+=elapsed_optimization;
  // ----------------------------------------------------------------------------------------
  auto t_start_missing_surface = std::chrono::high_resolution_clock::now();
  if (config.estimate_missing_surfaces && missing_surfaces != nullptr)
  {
    for (const auto &se : converged_superellipsoids)
    {
      missing_surfaces.push_back( se->estimateMissingSurfaces(config.missing_surfaces_threshold, config.missing_surfaces_num_samples) );
    }
  }

  double elapsed_missing_surface = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start_missing_surface).count();
  elapsed_total+=elapsed_missing_surface;

  // ----------------------------------------------------------------------------------------
  auto t_start_ros_messages = std::chrono::high_resolution_clock::now();

  // debug: publish superellipsoids for converged superellipsoids
  if (m_superellipsoids_pub.getNumSubscribers() > 0)
  {
    superellipsoid_msgs::SuperellipsoidArray sea;
    sea.header = pc_pcl_tf_ros_header;
    for (const auto &se : converged_superellipsoids)
    {
      superellipsoid_msgs::Superellipsoid se_msg = superellipsoid::toROSMsg(*se);
      se_msg.header = pc_pcl_tf_ros_header;
      sea.superellipsoids.push_back(se_msg);
    }
    m_superellipsoids_pub.publish(sea);
  }

  // debug: visualize prior cluster centers for converged superellipsoids
  if (m_centers_prior_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr prior_centers(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &se : converged_superellipsoids)
    {
      prior_centers->push_back(se->getEstimatedCenter());
      //ROS_INFO("%f %f %f", se->getEstimatedCenter().x, se->getEstimatedCenter().y, se->getEstimatedCenter().z);
    }
    sensor_msgs::PointCloud2::Ptr debug_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*prior_centers, *debug_pc_ros);
    debug_pc_ros->header = pc_pcl_tf_ros_header;
    m_centers_prior_pub.publish(debug_pc_ros);
  }

  // debug: visualize optimized centers for converged superellipsoids
  if (m_centers_optimized_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr optimized_centers(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &se : converged_superellipsoids)
    {
      optimized_centers->push_back(se->getOptimizedCenter());
    }
    sensor_msgs::PointCloud2::Ptr debug_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*optimized_centers, *debug_pc_ros);
    debug_pc_ros->header = pc_pcl_tf_ros_header;
    m_centers_optimized_pub.publish(debug_pc_ros);
  }

  // debug: visualize converged superellipsoids surface
  if (m_superellipsoids_surface_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr debug_pc(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &se : converged_superellipsoids)
    {
      *(debug_pc) += *(se->sampleSurface(m_config.use_fibonacci_sphere_projection_sampling));
    }
    sensor_msgs::PointCloud2::Ptr debug_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*debug_pc, *debug_pc_ros);
    debug_pc_ros->header = pc_pcl_tf_ros_header;
    m_superellipsoids_surface_pub.publish(debug_pc_ros);
  }

  // debug: visualize converged superellipsoids volume
  if (m_superellipsoids_volume_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr debug_pc(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &se : converged_superellipsoids)
    {
      *(debug_pc) += *(se->sampleVolume(m_config.pointcloud_volume_resolution));
    }
    sensor_msgs::PointCloud2::Ptr debug_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*debug_pc, *debug_pc_ros);
    debug_pc_ros->header = pc_pcl_tf_ros_header;
    m_superellipsoids_volume_pub.publish(debug_pc_ros);
  }

  // debug: visualize surface normals via rviz markers
  if (m_surface_normals_marker_pub.getNumSubscribers() > 0)
  {
    // DELETE ALL MARKERS
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.ns = "surface_normals_marker";
    marker.header = pc_pcl_tf_ros_header;
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);
    m_surface_normals_marker_pub.publish(marker_array);
    marker_array.markers.clear();

    // PUBLISH NORMALS FOR ALL POINTS
    int id_counter=0;
    for (const auto &se : converged_superellipsoids)
    {
      auto cloud_ = se->getCloud();
      auto normals_ = se->getNormals();

      for (int i=0; i<cloud_->points.size(); i++)
      {
        visualization_msgs::Marker marker;
        //marker.header.frame_id = m_config.world_frame;
        //marker.header.stamp = ros::Time();
        marker.header = pc_pcl_tf_ros_header;
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

  // debug: visualize xyz, label, normals, and curvature in a single message
  if (m_xyzlnormal_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr debug_pc(new pcl::PointCloud<pcl::PointXYZLNormal>);

    int id_counter = 0;
    for (const auto &se : converged_superellipsoids)
    {
      auto cloud_ = se->getCloud();
      auto normals_ = se->getNormals();
      debug_pc->points.reserve(debug_pc->points.size() + cloud_->points.size());

      for (int i = 0; i < cloud_->points.size(); i++)
      {
        pcl::PointXYZLNormal p;
        p.x = cloud_->points[i].x;
        p.y = cloud_->points[i].y;
        p.z = cloud_->points[i].z;
        p.label = id_counter;
        p.normal_x = normals_->points[i].normal_x;
        p.normal_y = normals_->points[i].normal_y;
        p.normal_z = normals_->points[i].normal_z;
        p.curvature = normals_->points[i].curvature;
        debug_pc->points.push_back(p);
      }
      id_counter++;
    }

    sensor_msgs::PointCloud2::Ptr debug_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*debug_pc, *debug_pc_ros);
    debug_pc_ros->header = pc_pcl_tf_ros_header;
    m_xyzlnormal_pub.publish(debug_pc_ros);
  }

  // debug: visualize missing surfaces
  if (m_missing_surfaces_pub.getNumSubscribers() > 0 && m_config.estimate_missing_surfaces)
  {
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr debug_pc(new pcl::PointCloud<pcl::PointXYZLNormal>);

    int id_counter = 0;
    for (int j = 0; j < converged_superellipsoids.size(); j++)
    {
      auto ms = missing_surfaces[j];
      auto oc = converged_superellipsoids[j]->getOptimizedCenter();
      debug_pc->points.reserve(debug_pc->points.size() + ms->points.size());

      for (int i = 0; i < ms->points.size(); i++)
      {
        pcl::PointXYZLNormal p;
        p.x = ms->points[i].x;
        p.y = ms->points[i].y;
        p.z = ms->points[i].z;
        p.label = id_counter;
        p.normal_x = oc.x - p.x;
        p.normal_y = oc.y - p.y;
        p.normal_z = oc.z - p.z;

        // normalize normal vector
        double normal_vec_len = sqrt(pow(p.normal_x,2) + pow(p.normal_y,2) + pow(p.normal_z,2));
        p.normal_x = p.normal_x / normal_vec_len;
        p.normal_y = p.normal_y / normal_vec_len;
        p.normal_z = p.normal_z / normal_vec_len;

        debug_pc->points.push_back(p);
      }
      id_counter++;
    }

    sensor_msgs::PointCloud2::Ptr debug_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*debug_pc, *debug_pc_ros);
    debug_pc_ros->header = pc_pcl_tf_ros_header;
    m_missing_surfaces_pub.publish(debug_pc_ros);
  }

  double elapsed_ros_messages = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start_ros_messages).count();
  elapsed_total+=elapsed_ros_messages;
  // ----------------------------------------------------------------------------------------
  
  ROS_INFO("Total Elapsed Time: %f ms", elapsed_total);
  ROS_INFO("(*) Preprocessing: %f ms", elapsed_preprocessing);
  ROS_INFO("(*) Clustering: %f ms", elapsed_clustering);
  ROS_INFO("(*) Optimization: %f ms", elapsed_optimization);
  ROS_INFO("(*) Missing Surface: %f ms", elapsed_missing_surface);
  ROS_INFO("(*) ROS Message Publish: %f ms", elapsed_ros_messages);
  ROS_WARN("====== Callback finished! =======");

}

} // namespace superellipsoid
