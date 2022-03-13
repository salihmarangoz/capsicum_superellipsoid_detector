

#ifndef SUPERELLIPSOID_H
#define SUPERELLIPSOID_H

#include <memory>
#include <vector>
#include <string>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <boost/math/special_functions/beta.hpp>

#ifndef DISABLE_ROS_SUPERELLIPSOID_H
#include <superellipsoid_msgs/Superellipsoid.h>
#endif

#ifndef SIGNUM
#define SIGNUM(x) ((x)>0?+1.:-1.)
#endif


namespace superellipsoid
{

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

template <typename PointT>
class Superellipsoid
{
public:
  Superellipsoid(typename pcl::PointCloud<PointT>::Ptr cloud_in);
  pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(float search_radius);
  pcl::PointXYZ estimateClusterCenter(float regularization);
  pcl::PointXYZ getEstimatedCenter();
  typename pcl::PointCloud<PointT>::Ptr getCloud();
  pcl::PointCloud<pcl::Normal>::Ptr getNormals();
  bool fit(bool log_to_stdout);

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// If you are creating the object with Superellipsoid ROS message then don't use the methods defined above unless specifying "cloud_in"
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#ifndef DISABLE_ROS_SUPERELLIPSOID_H
  Superellipsoid(const superellipsoid_msgs::Superellipsoid &se);
  superellipsoid_msgs::Superellipsoid generateRosMessage() const;
#endif

  pcl::PointCloud<pcl::PointXYZ>::Ptr sampleSurface(bool apply_transformation=true) const;
  pcl::PointCloud<pcl::PointXYZ>::Ptr sampleVolume(double resolution, bool apply_transformation=true) const;
  pcl::PointXYZ getOptimizedCenter() const;
  double computeVolume() const;
  std::map<std::string, double> getParameters() const;
  static double c_func(double w, double m);
  static double s_func(double w, double m);

private:
  typename pcl::PointCloud<PointT>::Ptr cloud_in;
  pcl::PointCloud<pcl::Normal>::Ptr normals_in;
  pcl::PointXYZ estimated_center;
  std::shared_ptr<std::vector<double>> parameters_ptr;
};


struct SuperellipsoidError {
  SuperellipsoidError(double x_, double y_, double z_, double *priors_);

  template <typename T> bool operator()(const T* const parameters, T* residual) const;

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double x,
                                     const double y,
                                     const double z,
                                     double* const priors) {
    return (new ceres::AutoDiffCostFunction<SuperellipsoidError, 3, 11>( // residual_size, parameters_size
        new SuperellipsoidError(x, y, z, priors)));
  }

 private:
  const double x;
  const double y;
  const double z;
  const double *priors;
};


template <typename PointT>
double Superellipsoid<PointT>::c_func(double w, double m) {return SIGNUM(cos(w)) * pow(abs(cos(w)), m);}


template <typename PointT>
double Superellipsoid<PointT>::s_func(double w, double m) {return SIGNUM(sin(w)) * pow(abs(sin(w)), m);}


template <typename PointT>
Superellipsoid<PointT>::Superellipsoid(typename pcl::PointCloud<PointT>::Ptr cloud_input)
{
  cloud_in = cloud_input;
  normals_in = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
  parameters_ptr = std::make_shared<std::vector<double>>();
}

#ifndef DISABLE_ROS_SUPERELLIPSOID_H
template <typename PointT>
Superellipsoid<PointT>::Superellipsoid(const superellipsoid_msgs::Superellipsoid &se)
{
  normals_in = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
  parameters_ptr = std::make_shared<std::vector<double>>();
  (*parameters_ptr)[0] = se.a;
  (*parameters_ptr)[1] = se.b;
  (*parameters_ptr)[2] = se.c;
  (*parameters_ptr)[3] = se.e1;
  (*parameters_ptr)[4] = se.e2;
  (*parameters_ptr)[5] = se.tx;
  (*parameters_ptr)[6] = se.ty;
  (*parameters_ptr)[7] = se.tz;
  (*parameters_ptr)[8] = se.roll;
  (*parameters_ptr)[9] = se.pitch;
  (*parameters_ptr)[10] = se.yaw;
}

template <typename PointT>
superellipsoid_msgs::Superellipsoid Superellipsoid<PointT>::generateRosMessage() const
{
  superellipsoid_msgs::Superellipsoid se;
  se.a = (*parameters_ptr)[0];
  se.b = (*parameters_ptr)[1];
  se.c = (*parameters_ptr)[2];
  se.e1 = (*parameters_ptr)[3];
  se.e2 = (*parameters_ptr)[4];
  se.tx = (*parameters_ptr)[5];
  se.ty = (*parameters_ptr)[6];
  se.tz = (*parameters_ptr)[7];
  se.roll = (*parameters_ptr)[8];
  se.pitch = (*parameters_ptr)[9];
  se.yaw = (*parameters_ptr)[10];
  se.volume = computeVolume();
  return se;
}
#endif


template <typename PointT>
pcl::PointCloud<pcl::Normal>::Ptr Superellipsoid<PointT>::estimateNormals(float search_radius)
{
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud_in);
  ne.setRadiusSearch (search_radius); // Use all neighbors in a sphere of radius 3cm (0.03f) might be a good value
  normals_in->points.reserve(cloud_in->points.size());
  ne.setViewPoint(estimated_center.x, estimated_center.y, estimated_center.z);
  ne.compute (*normals_in);
  return normals_in;
}


// Source: https://silo.tips/download/least-squares-intersection-of-lines
template <typename PointT>
pcl::PointXYZ Superellipsoid<PointT>::estimateClusterCenter(float regularization)
{
  // TODO: check cloud_in and normals_in have the same size

  Eigen::MatrixXf R_mat(3,3);
  R_mat.setZero();
  Eigen::MatrixXf Q_mat(3,1);
  Q_mat.setZero();
  auto eye3 = Eigen::MatrixXf::Identity(3, 3);
  Eigen::MatrixXf cluster_mean(3,1);
  cluster_mean.setZero();

  for (size_t i=0; i<normals_in->size(); i++)
  {
    auto normal_ = normals_in->at(i);
    auto point_ = cloud_in->at(i);

    Eigen::MatrixXf v(3,1);
    v << normal_.getNormalVector3fMap().x(), normal_.getNormalVector3fMap().y(), normal_.getNormalVector3fMap().z();

    Eigen::MatrixXf a(3,1);
    a << point_.getVector3fMap().x(), point_.getVector3fMap().y(), point_.getVector3fMap().z();

    R_mat += (eye3 - v * v.transpose());
    Q_mat += ((eye3 - (v * v.transpose() ) ) * a);

    cluster_mean += a;
  }

  cluster_mean = cluster_mean / normals_in->size();

  // regularization: bias towards a point (a value of 2.5 might be a good)
  R_mat += regularization * eye3;
  Q_mat += regularization * cluster_mean;

  auto R_dec = R_mat.completeOrthogonalDecomposition();
  auto R_inv = R_dec.pseudoInverse();
  auto cp_ = (R_inv * Q_mat).transpose();

  estimated_center = pcl::PointXYZ(cp_(0,0), cp_(0,1), cp_(0,2));

  if (std::isnan(estimated_center.x) || std::isnan(estimated_center.y) || std::isnan(estimated_center.z))
  {
    estimated_center = pcl::PointXYZ(cluster_mean(0,0), cluster_mean(1,0), cluster_mean(2,0));
    fprintf(stderr, "Center prediction with normals failed. Using cluster mean instead...\n");
  }
  return estimated_center;
}


template <typename PointT>
bool Superellipsoid<PointT>::fit(bool log_to_stdout)
{
  parameters_ptr->resize(16);
  auto parameters = (*parameters_ptr).data();

  auto priors_ptr = boost::make_shared<std::vector<double>>();
  priors_ptr->resize(16);
  auto priors = (*priors_ptr).data();

  parameters[0] = 0.05; // a
  parameters[1] = 0.05; // b
  parameters[2] = 0.05; // c
  parameters[3] = 0.5; // e1
  parameters[4] = 0.5; // e2
  parameters[5] = estimated_center._PointXYZ::x; // tx
  parameters[6] = estimated_center._PointXYZ::y; // ty
  parameters[7] = estimated_center._PointXYZ::z; // tz
  parameters[8] = 0.0; // roll
  parameters[9] = 0.0; // pitch
  parameters[10] = 0.0; //yaw

  priors[0] = estimated_center._PointXYZ::x; // tx
  priors[1] = estimated_center._PointXYZ::y; // ty
  priors[2] = estimated_center._PointXYZ::z; // tz

  Problem problem;
  for (size_t i=0; i<cloud_in->size(); i++) {
    auto point_ = cloud_in->at(i).getVector3fMap();
    CostFunction* cost_function = SuperellipsoidError::Create(point_.x(), point_.y(), point_.z(), priors);
    //problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.2), parameters); // loss todo
    problem.AddResidualBlock(cost_function, nullptr, parameters); // loss todo
  }

  // lower/upper bounds
  problem.SetParameterLowerBound(parameters, 0, 0.02); problem.SetParameterUpperBound(parameters, 0, 0.15); // a
  problem.SetParameterLowerBound(parameters, 1, 0.02); problem.SetParameterUpperBound(parameters, 1, 0.15); // b
  problem.SetParameterLowerBound(parameters, 2, 0.02); problem.SetParameterUpperBound(parameters, 2, 0.15); // c
  problem.SetParameterLowerBound(parameters, 3, 0.3); problem.SetParameterUpperBound(parameters, 3, 0.9); // e1
  problem.SetParameterLowerBound(parameters, 4, 0.3); problem.SetParameterUpperBound(parameters, 4, 0.9); // e2

  Solver::Options options;
  options.max_num_iterations = 100; // todo
  options.num_threads = 2; // can be good with SMT on modern CPU's
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.minimizer_progress_to_stdout = log_to_stdout;
  //options.check_gradients = true; // ???

  Solver::Summary summary;
  Solve(options, &problem, &summary);

  if (log_to_stdout)
  {
    std::cout << summary.BriefReport() << "\n";
    //std::cout << summary.FullReport() << "\n";
  }

  return summary.IsSolutionUsable(); // true if converged
}


// https://en.wikipedia.org/wiki/Superellipsoid
template <typename PointT>
pcl::PointCloud<pcl::PointXYZ>::Ptr Superellipsoid<PointT>::sampleSurface(bool apply_transformation/*=true*/) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_pc = (new pcl::PointCloud<pcl::PointXYZ>)->makeShared ();

  double a_ = (*parameters_ptr)[0];
  double b_ = (*parameters_ptr)[1];
  double c_ = (*parameters_ptr)[2];
  double e1_ = (*parameters_ptr)[3];
  double e2_ = (*parameters_ptr)[4];
  double tx_ = (*parameters_ptr)[5];
  double ty_ = (*parameters_ptr)[6];
  double tz_ = (*parameters_ptr)[7];
  double roll_ = (*parameters_ptr)[8];
  double pitch_ = (*parameters_ptr)[9];
  double yaw_ = (*parameters_ptr)[10];

  // rotation
  double angles[3] = {roll_, pitch_, yaw_};
  double rotation_matrix[9];
  ceres::EulerAnglesToRotationMatrix(angles, 3, rotation_matrix);
  auto rotation_matrix_ = Eigen::Map<Eigen::Matrix<double,3,3> >(rotation_matrix);

  for (double uu=-M_PI; uu<M_PI; uu+=0.05) // todo
  {
    for (double vv=-M_PI/2; vv<M_PI/2; vv+=0.05) // todo
    {
      double r = 2./e2_;
      double t = 2./e1_;

      double x = a_ * c_func(vv, 2./t) * c_func(uu, 2./r);
      double y = b_ * c_func(vv, 2./t) * s_func(uu, 2./r);
      double z = c_ * s_func(vv, 2./t);

      Eigen::Matrix<double, 3, 1> point {x,y,z};
      if (apply_transformation)
      {
        auto point_ = rotation_matrix_ * point;
        pcl::PointXYZ point__;
        point__.x = point_[0] + tx_;
        point__.y = point_[1] + ty_;
        point__.z = point_[2] + tz_;
        output_pc->push_back(point__);
      }
      else
      {
        pcl::PointXYZ point__;
        point__.x = point[0];
        point__.y = point[1];
        point__.z = point[2];
        output_pc->push_back(point__);
      }

    }
  }
  return output_pc;
}


// https://en.wikipedia.org/wiki/Superellipsoid
template <typename PointT>
pcl::PointCloud<pcl::PointXYZ>::Ptr Superellipsoid<PointT>::sampleVolume(double resolution, bool apply_transformation/*=true*/) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_pc = (new pcl::PointCloud<pcl::PointXYZ>)->makeShared ();

  double a_ = (*parameters_ptr)[0];
  double b_ = (*parameters_ptr)[1];
  double c_ = (*parameters_ptr)[2];
  double e1_ = (*parameters_ptr)[3];
  double e2_ = (*parameters_ptr)[4];
  double tx_ = (*parameters_ptr)[5];
  double ty_ = (*parameters_ptr)[6];
  double tz_ = (*parameters_ptr)[7];
  double roll_ = (*parameters_ptr)[8];
  double pitch_ = (*parameters_ptr)[9];
  double yaw_ = (*parameters_ptr)[10];

  // rotation
  double angles[3] = {roll_, pitch_, yaw_};
  double rotation_matrix[9];
  ceres::EulerAnglesToRotationMatrix(angles, 3, rotation_matrix);
  auto rotation_matrix_ = Eigen::Map<Eigen::Matrix<double,3,3> >(rotation_matrix);

  for (double xx=-a_; xx<=a_; xx+=resolution)
  {
    for (double yy=-b_; yy<=b_; yy+=resolution)
    {
      for (double zz=-c_; zz<=c_; zz+=resolution)
      {
        double r = 2./e2_;
        double t = 2./e1_;

        if (pow(pow(abs(xx/a_), r) + pow(abs(yy/b_), r), t/r) + pow(abs(zz/c_), t) <= 1) // the implicit inequality (is the point xx,yy,zz inside of the superellipsoid or not)
        {
          Eigen::Matrix<double, 3, 1> point {xx,yy,zz};
          if (apply_transformation)
          {
            auto point_ = rotation_matrix_ * point;
            pcl::PointXYZ point__;
            point__.x = point_[0] + tx_;
            point__.y = point_[1] + ty_;
            point__.z = point_[2] + tz_;
            output_pc->push_back(point__);
          }
          else
          {
            pcl::PointXYZ point__;
            point__.x = point[0];
            point__.y = point[1];
            point__.z = point[2];
            output_pc->push_back(point__);
          }

        }
      }
    }
  }

  return output_pc;
}


template <typename PointT>
std::map<std::string, double> Superellipsoid<PointT>::getParameters() const
{
  std::map<std::string, double> parameters_dict;
  parameters_dict.insert(std::pair<std::string, double>("a", (*parameters_ptr)[0]));
  parameters_dict.insert(std::pair<std::string, double>("b", (*parameters_ptr)[1]));
  parameters_dict.insert(std::pair<std::string, double>("c", (*parameters_ptr)[2]));
  parameters_dict.insert(std::pair<std::string, double>("e1", (*parameters_ptr)[3]));
  parameters_dict.insert(std::pair<std::string, double>("e2", (*parameters_ptr)[4]));
  parameters_dict.insert(std::pair<std::string, double>("tx", (*parameters_ptr)[5]));
  parameters_dict.insert(std::pair<std::string, double>("ty", (*parameters_ptr)[6]));
  parameters_dict.insert(std::pair<std::string, double>("tz", (*parameters_ptr)[7]));
  parameters_dict.insert(std::pair<std::string, double>("roll", (*parameters_ptr)[8]));
  parameters_dict.insert(std::pair<std::string, double>("pitch", (*parameters_ptr)[9]));
  parameters_dict.insert(std::pair<std::string, double>("yaw", (*parameters_ptr)[10]));

  return parameters_dict;
}


template <typename PointT>
pcl::PointXYZ Superellipsoid<PointT>::getOptimizedCenter() const
{
  pcl::PointXYZ p((*parameters_ptr)[5], (*parameters_ptr)[6], (*parameters_ptr)[7]);
  return p;
}


template <typename PointT>
pcl::PointXYZ Superellipsoid<PointT>::getEstimatedCenter()
{
  return estimated_center;
}


template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr Superellipsoid<PointT>::getCloud()
{
  return cloud_in;
}


template <typename PointT>
pcl::PointCloud<pcl::Normal>::Ptr Superellipsoid<PointT>::getNormals()
{
  return normals_in;
}


// ref: https://en.wikipedia.org/wiki/Superellipsoid
template <typename PointT>
double Superellipsoid<PointT>::computeVolume() const
{
  double a = (*parameters_ptr)[0];
  double b = (*parameters_ptr)[1];
  double c = (*parameters_ptr)[2];
  double e1 = (*parameters_ptr)[3];
  double e2 = (*parameters_ptr)[4];

  //double r = 2./e2;
  //double t = 2./e1;
  //return (2./3.) * a * b * c * (4./(r*t)) * boost::math::beta(1/r, 1/r) * boost::math::beta(2/t, 1/t);

  return (2./3.) * a * b * c * e1 * e2 * boost::math::beta(e2/2., e2/2.) * boost::math::beta(e1, e1/2.);
}


// ----------------------------------------------------------------------------------


inline SuperellipsoidError::SuperellipsoidError(double x_, double y_, double z_, double *priors_) : x(x_), y(y_), z(z_), priors(priors_){}


template <typename T> bool SuperellipsoidError::operator()(const T* const parameters, T* residual) const
{
  const T a = parameters[0];
  const T b = parameters[1];
  const T c = parameters[2];
  const T e1 = parameters[3];
  const T e2 = parameters[4];
  const T tx = parameters[5];
  const T ty = parameters[6];
  const T tz = parameters[7];
  const T roll = parameters[8];
  const T pitch = parameters[9];
  const T yaw = parameters[10];

  const double prior_tx = priors[0];
  const double prior_ty = priors[1];
  const double prior_tz = priors[2];

  // translation
  auto x_ = x - tx;
  auto y_ = y - ty;
  auto z_ = z - tz;

  // rotation
  const T angles[3] = {-roll, -pitch, -yaw};
  T rotation_matrix[9];
  ceres::EulerAnglesToRotationMatrix(angles, 3, rotation_matrix);
  auto rotation_matrix_ = Eigen::Matrix<T,3,3>(rotation_matrix);

  Eigen::Matrix<T, 3, 1> point {x_,y_,z_};

  auto point_ = rotation_matrix_ * point;

  const T x__ = point_[0];
  const T y__ = point_[1];
  const T z__ = point_[2];

  // loss
  const T f1 = pow(pow(abs(x__/a), 2./e2) + pow(abs(y__/b), 2./e2), e2/e1) + pow(abs(z__/c), 2./e1);
  //residual[0] = f1 - 1.; // doesn't fit well
  //residual[0] = sqrt(a*b*c) * (pow(f1,e1) - 1.); // cost function mentioned in Sweet Pepper Pose Detection and Grasping for Automated Crop Harvesting
  residual[0] = sqrt(pow(x__,2.)+pow(y__,2.)+pow(z__,2.)) * abs(1. - pow(f1, -e1/2.)); // Experimental: cost function based on distance to the surface mentioned here https://cse.buffalo.edu/~jryde/cse673/files/superquadrics.pdf

  // EXPERIMENTAL !!!!
  // regularization via prior
  const double C = 0.1;
  residual[1] =  C * sqrt(0.001 + pow(tx - prior_tx, 2) + pow(ty - prior_ty, 2) + pow(tz - prior_tz, 2));
  const double D = 0.1;
  residual[2] = D * sqrt(0.001 + pow(a-b, 2) + pow(b-c,2) + pow(c-a,2));
  

  /* EXPERIMENTAL (try to fit surface normals... FAILED. I NEED TO CHECK THE LITERATURE AGAIN. MAYBE HERE: https://cse.buffalo.edu/~jryde/cse673/files/superquadrics.pdf)
  auto u = atan2(y,x);
  auto v = 2.*asin(z);
  auto r = 2./e2;
  auto t = 2./e1;
  auto x___ = a * c_func_(v, 2./t) * c_func_(u, 2./r);
  auto y___ = b * c_func_(v, 2./t) * s_func_(u, 2./r);
  auto z___ = c * s_func_(v, 2./t);
  const double C = 0.1;
  residual[1] = (abs(x__)-x___)*C ;
  residual[2] = (abs(y__)-y___)*C ;
  residual[3] = (abs(z__)-z___)*C ;
  */

  return true;
}


} // namespace
#endif // SUPERELLIPSOID_H

