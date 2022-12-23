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
  m_pc_sub = m_nh.subscribe("pc_in", 1, &SuperellipsoidDetector::subscriberCallback, this); // queue size: 2 for throughput, 1 for low latency
  
  // ROS Service
  trigger_service = m_nh.advertiseService("trigger", &SuperellipsoidDetector::triggerCallback, this);
  
}

void SuperellipsoidDetector::startService()
{
  if (is_started)
  {
    ROS_ERROR("Node or service is already started. Only one can be started in a single process.");
  }
  is_started = true;

  // ROS Service
  fit_superellipsoids_service = m_nh.advertiseService("fit_superellipsoids", &SuperellipsoidDetector::serviceCallback, this);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////// TRIGGER CALLBACK //////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool SuperellipsoidDetector::triggerCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
{
  ROS_DEBUG("Triggered capsicum_superellipsoid_detector!");
  is_triggered = true;
  if (m_config.processing_mode != 1 /* ON_REQUEST */) ROS_WARN_THROTTLE(10.0, "The processing mode is CONTINUOUS. Triggering the node have no effect.");
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////// SERVICE CALLBACK //////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool SuperellipsoidDetector::serviceCallback(capsicum_superellipsoid_detector::FitSuperellipsoidsRequest& req, capsicum_superellipsoid_detector::FitSuperellipsoidsResponse& res)
{
  ROS_INFO("Request (for multiple superellipsoids) received...");

  ROS_WARN("====== Callback finished! =======");
  return true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////// TOPIC CALLBACK ////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void SuperellipsoidDetector::subscriberCallback(const sensor_msgs::PointCloud2Ptr &input_pc2)
{
  // Check processing mode
  if (m_config.processing_mode == 1 /* ON_REQUEST */ && !is_triggered) 
  {
    ROS_DEBUG("Skipped a message!");
    return;
  }
  is_triggered = false;

  // Optional Requests
  std::shared_ptr<std::vector<sensor_msgs::PointCloud2::Ptr>> missing_surfaces_rosmsgs;
  if (m_missing_surfaces_pub.getNumSubscribers()>0) missing_surfaces_rosmsgs.reset(new std::vector<sensor_msgs::PointCloud2::Ptr>);
  
  // Process input pointcloud
  std_msgs::Header new_header;
  superellipsoid::SuperellipsoidArray<pcl::PointXYZRGB> converged_superellipsoids;
  processInput(m_config, input_pc2, new_header, converged_superellipsoids, missing_surfaces_rosmsgs);

  // Visualize missing surfaces
  // Normal vectors should be directed toward optimized centroid. They are not real surface normals!
  if (missing_surfaces_rosmsgs != nullptr)
  {
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr missing_pc_pcl(new pcl::PointCloud<pcl::PointXYZLNormal>);

    int id_counter = 0;
    for (int j = 0; j < converged_superellipsoids.size(); j++)
    {
      auto ms = missing_surfaces_rosmsgs->at(j);
      auto oc = converged_superellipsoids[j].getOptimizedCenter();
      missing_pc_pcl->points.reserve(missing_pc_pcl->points.size() + ms->width*ms->height);

      for (sensor_msgs::PointCloud2ConstIterator<float> it(*ms, "x"); it != it.end(); ++it) {
        pcl::PointXYZLNormal p;
        p.x = it[0];
        p.y = it[1];
        p.z = it[2];
        p.label = id_counter;
        p.normal_x = oc.x - p.x;
        p.normal_y = oc.y - p.y;
        p.normal_z = oc.z - p.z;
        double normal_vec_len = sqrt(pow(p.normal_x,2) + pow(p.normal_y,2) + pow(p.normal_z,2));
        p.normal_x = p.normal_x / normal_vec_len;
        p.normal_y = p.normal_y / normal_vec_len;
        p.normal_z = p.normal_z / normal_vec_len;

        missing_pc_pcl->points.push_back(p);
      }

      id_counter++;
    }

    sensor_msgs::PointCloud2::Ptr missing_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*missing_pc_pcl, *missing_pc_ros);
    missing_pc_ros->header = input_pc2->header; // TODO: use new header
    m_missing_surfaces_pub.publish(missing_pc_ros);
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////// PROCESSING SERVICE AND TOPIC CALLBACKS HAPPENING IN THE METHOD IN THE BIG CHUNK BELOW /////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SuperellipsoidDetector::processInput(capsicum_superellipsoid_detector::SuperellipsoidDetectorConfig &config,
                                          const sensor_msgs::PointCloud2::ConstPtr &pc2,
                                          std_msgs::Header &new_header,
                                          std::vector<superellipsoid::Superellipsoid<pcl::PointXYZRGB>> &converged_superellipsoids,
                                          std::shared_ptr<std::vector<sensor_msgs::PointCloud2::Ptr>> &missing_surfaces
                                          )
{
  auto t_start = std::chrono::high_resolution_clock::now();
  ROS_INFO("Pointcloud received...");

  // Convert to PCL Pointcloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*pc2, *pc_pcl);

  // Filter Invalid Points
  std::shared_ptr<std::vector<int>> indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*pc_pcl, *pc_pcl, *indices);

  if (pc_pcl->size() < config.min_cluster_size)
  {
    ROS_WARN("Number of points (%lu) is smaller than min_cluster_size (%d). Aborting...", pc_pcl->size(), config.min_cluster_size);
    return;
  }

  // Transform pointcloud to the world frame
  pcl_ros::transformPointCloud(config.world_frame, *pc_pcl, *pc_pcl, *m_tf_listener);
  new_header = pcl_conversions::fromPCL(pc_pcl->header); // <-------- use this header for output messages!

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

  if (missing_surfaces != nullptr)
  {
    for (auto &se : converged_superellipsoids)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr ms_pcl = se.estimateMissingSurfaces(config.missing_surfaces_threshold, config.missing_surfaces_num_samples);
      sensor_msgs::PointCloud2::Ptr ms_ros(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*ms_pcl, *ms_ros);
      ms_ros->header = new_header;
      missing_surfaces->push_back(ms_ros);
    }
  }

  // ----------------------------------------------------------------------------------------

  // debug: publish superellipsoids for converged superellipsoids
  /*
  if (m_superellipsoids_pub.getNumSubscribers() > 0)
  {
    superellipsoid_msgs::SuperellipsoidArray sea;
    sea.header = new_header;
    for (const auto &se : converged_superellipsoids)
    {
      superellipsoid_msgs::Superellipsoid se_msg = superellipsoid::toROSMsg(*se);
      se_msg.header = new_header;
      sea.superellipsoids.push_back(se_msg);
    }
    m_superellipsoids_pub.publish(sea);
  }
  */

  // debug: visualize prior cluster centers for converged superellipsoids
  /*
  if (m_centers_prior_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr prior_centers(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &se : converged_superellipsoids)
    {
      prior_centers->push_back(se->getEstimatedCenter());
    }
    sensor_msgs::PointCloud2::Ptr debug_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*prior_centers, *debug_pc_ros);
    debug_pc_ros->header = new_header;
    m_centers_prior_pub.publish(debug_pc_ros);
  }
  */

  // debug: visualize optimized centers for converged superellipsoids
  /*
  if (m_centers_optimized_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr optimized_centers(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &se : converged_superellipsoids)
    {
      optimized_centers->push_back(se->getOptimizedCenter());
    }
    sensor_msgs::PointCloud2::Ptr debug_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*optimized_centers, *debug_pc_ros);
    debug_pc_ros->header = new_header;
    m_centers_optimized_pub.publish(debug_pc_ros);
  }
  */

  // debug: visualize converged superellipsoids surface
  /*
  if (m_superellipsoids_surface_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr debug_pc(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &se : converged_superellipsoids)
    {
      *(debug_pc) += *(se->sampleSurface(config.use_fibonacci_sphere_projection_sampling));
    }
    sensor_msgs::PointCloud2::Ptr debug_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*debug_pc, *debug_pc_ros);
    debug_pc_ros->header = new_header;
    m_superellipsoids_surface_pub.publish(debug_pc_ros);
  }
  */

  // debug: visualize converged superellipsoids volume
  /*
  if (m_superellipsoids_volume_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr debug_pc(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &se : converged_superellipsoids)
    {
      *(debug_pc) += *(se->sampleVolume(config.pointcloud_volume_resolution));
    }
    sensor_msgs::PointCloud2::Ptr debug_pc_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*debug_pc, *debug_pc_ros);
    debug_pc_ros->header = new_header;
    m_superellipsoids_volume_pub.publish(debug_pc_ros);
  }
  */

  // debug: visualize surface normals via rviz markers
  /*
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

    // PUBLISH NORMALS FOR ALL POINTS
    int id_counter=0;
    for (const auto &se : converged_superellipsoids)
    {
      auto cloud_ = se->getCloud();
      auto normals_ = se->getNormals();

      for (int i=0; i<cloud_->points.size(); i++)
      {
        visualization_msgs::Marker marker;
        //marker.header.frame_id = config.world_frame;
        //marker.header.stamp = ros::Time();
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
  */

  // debug: visualize xyz, label, normals, and curvature in a single message
  /*
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
    debug_pc_ros->header = new_header;
    m_xyzlnormal_pub.publish(debug_pc_ros);
  }
  */

  double elapsed_time = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
  ROS_INFO("Total Elapsed Time: %f ms", elapsed_time);
  ROS_WARN("====== Callback finished! =======");
}

} // namespace superellipsoid
