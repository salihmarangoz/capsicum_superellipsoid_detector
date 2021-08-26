#include <ros/ros.h>
//#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>

#include "ceres/ceres.h"
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


struct SuperellipsoidError {
  SuperellipsoidError(double x, double y, double z)
      : x_(x), y_(y), z_(z) {}

  template <typename T> bool operator()(const T* const parameters,
                                        T* residual) const {
    const T a_ = parameters[0];
    const T b_ = parameters[1];
    const T c_ = parameters[2];
    const T e1_ = parameters[3];
    const T e2_ = parameters[4];
    const T tx_ = parameters[5];
    const T ty_ = parameters[6];
    const T tz_ = parameters[7];

    // constraints (TODO)
    auto a = abs(a_) + 0.00001;
    auto b = abs(b_) + 0.00001;
    auto c = abs(c_) + 0.00001;
    auto e1 = abs(e1_) + 0.1;
    auto e2 = abs(e2_) + 0.1;

    // translation
    auto x = abs(x_ - tx_);
    auto y = abs(y_ - tx_);
    auto z = abs(z_ - tx_);

    //ROS_INFO("debug: %f %f %f", x, x_, tx);

    auto f1 = pow(pow(x/a, 2./e2) + pow(y/b, 2./e2), e2/e1) + pow(z/c, 2./e1);
    residual[0] = sqrt(a*b*c) * (pow(f1,e1) - 1.);
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double x,
                                     const double y,
                                     const double z) {
    return (new ceres::AutoDiffCostFunction<SuperellipsoidError, 1, 8>(
        new SuperellipsoidError(x, y, z)));
  }

 private:
  const double x_;
  const double y_;
  const double z_;
};



// globals
ros::Publisher pc_roi_pub, pc_other_pub, clusters_pub, centers_pub, vis_pub, superellipsoid_pub;
std::unique_ptr<tf::TransformListener> listener;



#define SIGNUM(x) ((x)>0?+1.:-1.)

double c_func(double w, double m)
{
  return SIGNUM(cos(w)) * pow(abs(cos(w)), m);
}

double s_func(double w, double m)
{
  return SIGNUM(sin(w)) * pow(abs(sin(w)), m);
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

pcl::PointXYZ estimate_cluster_center(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, float search_radius=0.03, float regularization=2.5)
{
  // Estimate Normals
  // TODO: This would be faster, maybe: Randomly sample some points or apply clustering then estimate normals
  // Currently using all points
  pcl::PointCloud<pcl::Normal>::Ptr pc_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod(tree);
  ne.setInputCloud(pc);
  ne.setRadiusSearch (search_radius); // Use all neighbors in a sphere of radius 3cm
  ne.compute (*pc_normals);

  Eigen::MatrixXf R_mat(3,3);
  R_mat.setZero();
  Eigen::MatrixXf Q_mat(3,1);
  Q_mat.setZero();
  auto eye3 = Eigen::MatrixXf::Identity(3, 3);
  Eigen::MatrixXf cluster_mean(3,1);
  cluster_mean.setZero();

  for (size_t i=0; i<pc_normals->size(); i++)
  {
    auto normal_ = pc_normals->at(i);
    auto point_ = pc->at(i);

    Eigen::MatrixXf v(3,1);
    v << normal_.getNormalVector3fMap().x(), normal_.getNormalVector3fMap().y(), normal_.getNormalVector3fMap().z();

    Eigen::MatrixXf a(3,1);
    a << point_.getVector3fMap().x(), point_.getVector3fMap().y(), point_.getVector3fMap().z();

    R_mat += (eye3 - v * v.transpose());
    Q_mat += ((eye3 - (v * v.transpose() ) ) * a);

    cluster_mean += a;
  }

  cluster_mean = cluster_mean / pc_normals->size();

  // experimental regularization
  R_mat += regularization * eye3;
  Q_mat += + regularization * cluster_mean;

  auto R_dec = R_mat.completeOrthogonalDecomposition();
  auto R_inv = R_dec.pseudoInverse();
  auto cp_ = (R_inv * Q_mat).transpose();
  return pcl::PointXYZ(cp_(0,0), cp_(0,1), cp_(0,2));
}


// ======================================================================================================================================
void pc_callback(const sensor_msgs::PointCloud2Ptr& pc_ros)
{
  ROS_INFO_ONCE("Pointcloud received...");


  // Convert to PCL Pointcloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*pc_ros, *pc_pcl);


  // Filter NaN Values
  std::shared_ptr<std::vector<int>> indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*pc_pcl, *pc_pcl, *indices);


  // Transform pointcloud to the world frame
  pcl_ros::transformPointCloud("world", *pc_pcl, *pc_pcl, *listener);
  std_msgs::Header pc_pcl_tf_ros_header = pcl_conversions::fromPCL(pc_pcl->header);
  if (pc_pcl->size() == 0)
  {
    ROS_WARN("pc_pcl_tf->size() == 0");
    return;
  }


  // Seperate pointclouds to roi & non-roi using color information
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
  pcl::IndicesConstPtr removed_indices_ = condrem.getRemovedIndices();

  for (auto it = std::begin(*removed_indices_); it!=std::end(*removed_indices_); ++it)
  {
    pcl::PointXYZRGB point(pc_pcl->at(*it));
    roi_pc->push_back(point);
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


  // Predict roi centers then fit superellipsoids
  pcl::PointCloud<pcl::PointXYZ>::Ptr roi_centers (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr superellipsoids (new pcl::PointCloud<pcl::PointXYZ>);

  for (const auto& i_segment : cluster_indices)
  {
    // Gather points for a segment
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_tmp_ (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& i_point : (i_segment.indices))
    {
      pc_tmp_->push_back(roi_pc->at(i_point));
    }

    auto cp_pcl = estimate_cluster_center(pc_tmp_);
    roi_centers->push_back(cp_pcl);

    //-------------------------------------------------

    // Fit superellipsoid using roi_centers
    double parameters[128];
    parameters[0] = 0.05; // a
    parameters[1] = 0.05; // b
    parameters[2] = 0.05; // c
    parameters[3] = 0.5; // e1
    parameters[4] = 0.5; // e2
    parameters[5] = cp_pcl._PointXYZ::x;
    parameters[6] = cp_pcl._PointXYZ::y;
    parameters[7] = cp_pcl._PointXYZ::z;

    Problem problem;
    for (size_t i=0; i<pc_tmp_->size(); i++) {
      auto point_ = pc_tmp_->at(i).getVector3fMap();

      CostFunction* cost_function = SuperellipsoidError::Create(point_.x(), point_.y(), point_.z());

      problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), parameters);
    }

    Solver::Options options;
    options.max_num_iterations = 100; // todo
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = false;

    Solver::Summary summary;
    Solve(options, &problem, &summary);

    if (!summary.IsSolutionUsable()) // report if the solution is not usable
      std::cout << summary.BriefReport() << "\n";


    // // https://en.wikipedia.org/wiki/Superellipsoid
    if (superellipsoid_pub.getNumSubscribers() > 0)
    {
      double a_ = parameters[0];
      double b_ = parameters[1];
      double c_ = parameters[2];
      double e1_ = parameters[3];
      double e2_ = parameters[4];
      double tx_ = parameters[5];
      double ty_ = parameters[6];
      double tz_ = parameters[7];

      for (double uu=-M_PI/2; uu<M_PI/2; uu+=0.1)
      {
        for (double vv=-M_PI; vv<M_PI; vv+=0.2)
        {
          double r = 2/e2_;
          double t = 2/e1_;

          double x = a_ * c_func(vv, 2./t) * c_func(uu, 2./r);
          double y = b_ * c_func(vv, 2./t) * s_func(uu, 2./r);
          double z = c_ * s_func(vv, 2./t);

          x = x + tx_;
          y = y + ty_;
          z = z + tz_;

          pcl::PointXYZ point;
          point.x = x;
          point.y = y;
          point.z = z;
          superellipsoids->push_back(point);
        }
      }
    }


  }


  if (pc_roi_pub.getNumSubscribers() > 0 )
  {
    sensor_msgs::PointCloud2::Ptr roi_pc2_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*roi_pc, *roi_pc2_ros);
    roi_pc2_ros->header = pc_pcl_tf_ros_header;
    pc_roi_pub.publish(roi_pc2_ros);
  }

  if (pc_other_pub.getNumSubscribers() > 0 )
  {
    sensor_msgs::PointCloud2::Ptr other_pc2_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*other_pc, *other_pc2_ros);
    other_pc2_ros->header = pc_pcl_tf_ros_header;
    pc_other_pub.publish(other_pc2_ros);
  }

  if (clusters_pub.getNumSubscribers() > 0 )
  {
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = getColoredCloud (roi_pc, cluster_indices);

    sensor_msgs::PointCloud2::Ptr debug_pc(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*colored_cloud, *debug_pc);
    debug_pc->header = pc_pcl_tf_ros_header;
    clusters_pub.publish(debug_pc);
  }

  if (superellipsoid_pub.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2::Ptr tmp_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*superellipsoids, *tmp_ros);
    tmp_ros->header = pc_pcl_tf_ros_header;
    superellipsoid_pub.publish(tmp_ros);
  }

  if (centers_pub.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2::Ptr roi_centers_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*roi_centers, *roi_centers_ros);
    roi_centers_ros->header = pc_pcl_tf_ros_header;
    centers_pub.publish(roi_centers_ros);
  }

  ROS_WARN("Callback finished!");
}


int
main(int argc, char **argv)
{
  ros::init(argc, argv, "capsicum_superellipsoid_detector_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  listener = std::make_unique<tf::TransformListener>();
  pc_roi_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("pc_roi_out", 2);
  pc_other_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("pc_other_out", 2);
  clusters_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("clusters_out", 2);
  centers_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("centers_out", 2);
  vis_pub = priv_nh.advertise<visualization_msgs::Marker>( "visualization_marker", 2);
  superellipsoid_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("superellipsoid_out", 2);

  ros::Subscriber pc_sub = nh.subscribe("pc_in", 10, pc_callback);

  ros::spin();
  return 0;
}
