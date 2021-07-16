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

/*
def loss(x0,x,y,z):
    a,b,c,e1,e2,tx,ty,tz,yaw,pitch,roll = x0

    # https://otik.uk.zcu.cz/bitstream/11025/1637/1/D71.pdf
    x,y,z = rotate_xyz(x,y,z,roll,pitch,yaw)
    x,y,z = translate_xyz(x,y,z,tx,ty,tz)

    #x,y,z = x+0j, y+0j, z+0j
    x,y,z = np.abs(x), np.abs(y), np.abs(z)

    f = ( (x/a)**(2/e2) + (y/b)**(2/e2) )**(e2/e1) + (z/c)**(2/e1)
    #f = (np.sqrt(a*b*c) * (f**e1 - 1.))**2
    f = np.abs(a*b*c) * (f**e1 - 1.)**2

    # additions: (MAYBE? OR ADD THESE AS A CONSTRAINT)
    #f += 0.001*np.abs(tx + ty + tz)
    #f += 0.001*np.abs(roll+pitch+yaw)
    #f += 0.001*(a-1)
    #f += 0.001*(b-1)
    #f += 0.001*(c-1)

    #f = np.sum(f)
    #f = np.linalg.norm(f)
    #print(f)
    return f
*/
/*
struct SuperellipsoidCostFunctor {
  SuperellipsoidCostFunctor(const double x, const double y, const double z) : x_(x), y_(y), z_(z) {}

  template <typename T>
  bool operator()(const T* parameters, T* residuals) const {
    const T a = parameters[0];
    const T b = parameters[1];
    const T c = parameters[2];
    const T e1 = parameters[3];
    const T e2 = parameters[4];
    const T tx = parameters[5];
    const T ty = parameters[6];
    const T tz = parameters[7];
    const T yaw = parameters[8];
    const T pitch = parameters[9];
    const T roll = parameters[10];

    residuals[0] = b1 * pow(1.0 + exp(b2 -  b3 * x_), -1.0 / b4) - y_;
    return true;
  }

  private:
    const double x_;
    const double y_;
    const double z_;
};
*/

// globals
ros::Publisher pc_roi_pub, pc_other_pub, clusters_pub, centers_pub, vis_pub;
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
    //ROS_INFO("%d", roi_pc->points.size());
    pc_roi_pub.publish(roi_pc2_ros);
  }
  if (pc_other_pub.getNumSubscribers() > 0 )
  {
    sensor_msgs::PointCloud2::Ptr other_pc2_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*other_pc, *other_pc2_ros);
    other_pc2_ros->header = pc_pcl_tf_ros_header;
    //ROS_INFO("%d", other_pc->points.size());
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

  if (clusters_pub.getNumSubscribers() > 0 )
  {
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = getColoredCloud (roi_pc, cluster_indices);

    sensor_msgs::PointCloud2::Ptr debug_pc(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*colored_cloud, *debug_pc);
    debug_pc->header = pc_pcl_tf_ros_header;
    //ROS_INFO("%d", colored_cloud->points.size());
    clusters_pub.publish(debug_pc);
  }


  // Predict roi centers
  pcl::PointCloud<pcl::PointXYZ>::Ptr roi_centers (new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& i_segment : cluster_indices)
  {
    // Gather points for a segment
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_tmp_ (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& i_point : (i_segment.indices))
    {
      pc_tmp_->push_back((*roi_pc)[i_point]);
    }

    // TODO: Randomly sample some points or apply clustering then estimate normals
    // Estimate Normals
    pcl::PointCloud<pcl::Normal>::Ptr pc_tmp_normals_ (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod(tree);
    ne.setInputCloud(pc_tmp_);
    ne.setRadiusSearch (0.03); // Use all neighbors in a sphere of radius 3cm
    ne.compute (*pc_tmp_normals_);

    /* Note: pc_tmp_normals_ vectors are normalized!
    for (const auto& p_ : *pc_tmp_normals_)
    {
      float vx = p_._Normal::normal[0];
      float vy = p_._Normal::normal[1];
      float vz = p_._Normal::normal[2];
      ROS_INFO_STREAM(sqrt(vx*vx + vy*vy + vz*vz));
    }*/

    Eigen::MatrixXf R_mat(3,3);
    R_mat.setZero();
    Eigen::MatrixXf Q_mat(3,1);
    Q_mat.setZero();
    auto eye3 = Eigen::MatrixXf::Identity(3, 3);

    for (size_t i=0; i<pc_tmp_normals_->size(); i++)
    {
      auto normal_ = pc_tmp_normals_->at(i);
      auto point_ = pc_tmp_->at(i);
      //ROS_INFO_STREAM("-----");

      //auto v = Eigen::MatrixXf::Map(normal_._Normal::normal, 1, 3);
      Eigen::MatrixXf v(3,1);
      v << normal_.getNormalVector3fMap().x(), normal_.getNormalVector3fMap().y(), normal_.getNormalVector3fMap().z();
      //ROS_INFO_STREAM(v);

      //auto a = Eigen::MatrixXf::Map(point_.getVector3fMap().data(), 3, 1);
      Eigen::MatrixXf a(3,1);
      a << point_.getVector3fMap().x(), point_.getVector3fMap().y(), point_.getVector3fMap().z();
      //ROS_INFO_STREAM(a);

      R_mat += (eye3 - v * v.transpose());
      //ROS_INFO_STREAM(std::endl << eye3 - v * v.transpose());
      Q_mat += ((eye3 - (v * v.transpose() ) ) * a);
      //ROS_INFO_STREAM(std::endl << a);
      //ROS_INFO_STREAM(std::endl << (eye3 - v * v.transpose()) * a);

    }

    //ROS_INFO_STREAM("R_mat:" << std::endl << R_mat << std::endl << "Q_mat:" << std::endl << Q_mat);
    //ROS_INFO_STREAM(R_mat.rows() << " " << R_mat.cols() << " " << Q_mat.rows() << " " << Q_mat.cols());

    //auto cp_ = R_mat.completeOrthogonalDecomposition().solve(Q_mat.transpose());

    auto R_dec = R_mat.completeOrthogonalDecomposition();
    //auto cp_ = R_dec.solve(Q_mat);
    auto R_inv = R_dec.pseudoInverse();
    auto cp_ = (R_inv * Q_mat).transpose();

    //ROS_INFO_STREAM("cp: " << cp_);
    //ROS_INFO_STREAM("cp: " << cp_(0,0) << " " << cp_(0,1) << " " << cp_(0,2));
    auto cp_pcl = pcl::PointXYZ(cp_(0,0), cp_(0,1), cp_(0,2));
    roi_centers->push_back(cp_pcl);


  }
/*
  Eigen::Matrix3f A;
  Eigen::Vector3f b;
  A << 1,2,3,  4,5,6,  7,8,10;
  b << 3, 3, 4;
  Eigen::Vector3f x = A.colPivHouseholderQr().solve(b);
  ROS_INFO_STREAM(x);
*/

  sensor_msgs::PointCloud2::Ptr roi_centers_ros(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*roi_centers, *roi_centers_ros);
  roi_centers_ros->header = pc_pcl_tf_ros_header;
  centers_pub.publish(roi_centers_ros);


  // Fit superellipsoid using roi_centers

/*
  CostFunction* cost_function =
        new AutoDiffCostFunction<Rat43CostFunctor, 1, 4>(
          new Rat43CostFunctor(x, y));


    double x = 0.5;
    const double initial_x = x;
    // Build the problem.
    Problem problem;
    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    CostFunction* cost_function =
        new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    problem.AddResidualBlock(cost_function, nullptr, &x);
    // Run the solver!
    Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    ROS_INFO_STREAM(summary.BriefReport());
*/



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
  centers_pub = priv_nh.advertise<sensor_msgs::PointCloud2>("centers_out", 2);
  vis_pub = priv_nh.advertise<visualization_msgs::Marker>( "visualization_marker", 2);

  ros::Subscriber pc_sub = nh.subscribe("pc_in", 1, pc_callback);

  ros::spin();
  return 0;
}
