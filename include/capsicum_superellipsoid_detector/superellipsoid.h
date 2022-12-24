#ifndef SUPERELLIPSOID_H
#define SUPERELLIPSOID_H

#include <cmath>
#include <memory>
#include <vector>
#include <string>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <boost/math/special_functions/beta.hpp>
#include <boost/math/special_functions/sign.hpp>

namespace superellipsoid
{

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

enum CostFunctionType{
  NAIVE=0,
  LEHNERT=1,
  RADIAL_EUCLIDIAN_DISTANCE=2,
  SOLINA=3,
  SOLINA_DISTANCE=4
};

//////////////////////////////////////////////////////////////////
//////// SUPERELLIPSOID ERROR ////////////////////////////////////
//////////////////////////////////////////////////////////////////

struct SuperellipsoidError {
  SuperellipsoidError(double x_, double y_, double z_, double *priors_, CostFunctionType cost_type_, double prior_center_, double prior_scaling_);

  template <typename T> bool operator()(const T* const parameters, T* residual) const;

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double x,
                                     const double y,
                                     const double z,
                                     double* const priors,
                                     const CostFunctionType cost_type,
                                     const double prior_center,
                                     const double prior_scaling) {
    return (new ceres::AutoDiffCostFunction<SuperellipsoidError, 3, 11>( // residual_size, parameters_size
        new SuperellipsoidError(x, y, z, priors, cost_type, prior_center, prior_scaling)));
  }

 private:
  const double x;
  const double y;
  const double z;
  const double *priors;
  const CostFunctionType cost_type;
  const double prior_center;
  const double prior_scaling;
};

inline SuperellipsoidError::SuperellipsoidError(double x_, double y_, double z_, double *priors_, CostFunctionType cost_type_, double prior_center_, double prior_scaling_) : x(x_), y(y_), z(z_), priors(priors_), cost_type(cost_type_), prior_center(prior_center_), prior_scaling(prior_scaling_) {}

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

  /////////////////////////// Cost /////////////////////////////////////
  const T f1 = pow(pow(abs(x__/a), 2./e2) + pow(abs(y__/b), 2./e2), e2/e1) + pow(abs(z__/c), 2./e1);
  switch (cost_type)
  {
    case CostFunctionType::NAIVE: // Naive solution using implicit definition of the superellipsoid
      residual[0] = f1 - 1.;
      break;
    case CostFunctionType::LEHNERT: // Cost function mentioned in paper; "Sweet Pepper Pose Detection and Grasping for Automated Crop Harvesting"
      residual[0] = sqrt(a*b*c) * (pow(f1,e1) - 1.);
      break;
    case CostFunctionType::RADIAL_EUCLIDIAN_DISTANCE: // Cost function based on distance to the surface approximation (by radial euclidian distances) mentioned here https://cse.buffalo.edu/~jryde/cse673/files/superquadrics.pdf
      residual[0] = sqrt(pow(x__,2.)+pow(y__,2.)+pow(z__,2.)) * abs(1. - pow(f1, -e1/2.));
      break;
    case CostFunctionType::SOLINA: // Squared version of the cost function mentioned in "Sweet Pepper Pose Detection and Grasping for Automated Crop Harvesting"
      residual[0] = sqrt(a*b*c) * abs(pow(f1,e1/2.) - 1.);
      break;
    case CostFunctionType::SOLINA_DISTANCE: // SOLINA without volume constraint ( sqrt(abc) )
      residual[0] = abs(pow(f1,e1/2.) - 1.);
      break;
  }

  /////////////////////////// Priors /////////////////////////////////////
  residual[1] = prior_center * sqrt(0.001 + pow(tx - prior_tx, 2) + pow(ty - prior_ty, 2) + pow(tz - prior_tz, 2));
  residual[2] = prior_scaling * sqrt(0.001 + pow(a - b, 2) + pow(b - c,2) + pow(c - a,2));

  return true;
}

//////////////////////////////////////////////////////////////////
//////// SUPERELLIPSOID DECLERATION //////////////////////////////
//////////////////////////////////////////////////////////////////

template <typename PointT>
class Superellipsoid
{
public:
  // TODO: set const better!
  // TODO: constructor without cloud_in
  Superellipsoid(typename pcl::PointCloud<PointT>::Ptr cloud_in);
  pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(float search_radius);
  pcl::PointXYZ estimateClusterCenter(float regularization, bool flip_normals_towards_estimated_center=true);
  const typename pcl::PointCloud<PointT>::Ptr getCloud() const;
  const pcl::PointCloud<pcl::Normal>::Ptr getNormals() const;
  bool fit(bool log_to_stdout=true, int max_num_iterations=100, CostFunctionType cost_type=CostFunctionType::RADIAL_EUCLIDIAN_DISTANCE, double prior_center=0.1, double prior_scaling=0.1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr estimateMissingSurfaces(double distance_threshold=0.01, int num_samples=1000);
  pcl::PointCloud<pcl::PointXYZ>::Ptr _sampleSurfaceFibonacciProjection(double a, double b, double c, double e1, double e2, double tx, double ty, double tz, double roll, double pitch, double yaw, int num_samples=1000) const;
  pcl::PointCloud<pcl::PointXYZ>::Ptr _sampleSurfaceParametricDefinition(double a, double b, double c, double e1, double e2, double tx, double ty, double tz, double roll, double pitch, double yaw, double u_res=0.05, double v_res=0.05) const;
  pcl::PointCloud<pcl::PointXYZ>::Ptr sampleSurface(bool use_fibonacci_projection=false, bool apply_transformation=true, int num_samples_fibonacci=1000, double u_res_parametric=0.05, double v_res_parametric=0.05) const;
  pcl::PointCloud<pcl::PointXYZ>::Ptr sampleVolume(double resolution, bool apply_transformation=true) const;
  pcl::PointXYZ getOptimizedCenter() const;
  pcl::PointXYZ getEstimatedCenter() const;
  double computeVolume() const;
  std::map<std::string, double> getParameters() const;
  static double c_func(double w, double m);
  static double s_func(double w, double m);

//private:
  typename pcl::PointCloud<PointT>::Ptr cloud_in;
  pcl::PointCloud<pcl::Normal>::Ptr normals_in;
  pcl::PointXYZ estimated_center;
  std::shared_ptr<std::vector<double>> parameters_ptr;
};

template <typename PointT>
Superellipsoid<PointT>::Superellipsoid(typename pcl::PointCloud<PointT>::Ptr cloud_input)
{
  cloud_in = cloud_input;
  normals_in = pcl::make_shared<pcl::PointCloud<pcl::Normal>>();
  parameters_ptr = std::make_shared<std::vector<double>>();
}

//////////////////////////////////////////////////////////////////
//////// SUPERELLIPSOID CAPSULATED TYPES /////////////////////////
//////////////////////////////////////////////////////////////////

// Superellipsoid -> encapsulated by a pointer
template <typename PointT>
using SuperellipsoidPtr = std::shared_ptr<Superellipsoid<PointT>>;

// Superellipsoid -> inside a vector
template <typename PointT>
using SuperellipsoidArray = std::vector<Superellipsoid<PointT>>;

// Superellipsoid -> inside a vector -> encapsulated by a pointer
template <typename PointT>
using SuperellipsoidArrayPtr = std::shared_ptr<SuperellipsoidArray<PointT>>;


//////////////////////////////////////////////////////////////////
//////// SUPERELLIPSOID DEFINITION ///////////////////////////////
//////////////////////////////////////////////////////////////////

template <typename PointT>
double Superellipsoid<PointT>::c_func(double w, double m) {return std::copysign(pow(abs(cos(w)), m) , cos(w));}


template <typename PointT>
double Superellipsoid<PointT>::s_func(double w, double m) {return std::copysign(pow(abs(sin(w)), m) , sin(w));}


template <typename PointT>
pcl::PointCloud<pcl::Normal>::Ptr Superellipsoid<PointT>::estimateNormals(float search_radius)
{
  if (pcl::traits::has_normal_v<PointT>)
  {
    printf("Using given normals is not implemented yet. Estimating normals...\n");
    // TODO: use input normals instead if available
  }

  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud_in);
  ne.setRadiusSearch (search_radius); // Use all neighbors in a sphere of radius 3cm (0.03f) might be a good value
  normals_in->points.reserve(cloud_in->points.size());
  ne.compute (*normals_in);

  return normals_in;
}

// Source: https://silo.tips/download/least-squares-intersection-of-lines
template <typename PointT>
pcl::PointXYZ Superellipsoid<PointT>::estimateClusterCenter(float regularization, bool flip_normals_towards_estimated_center)
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

  if (flip_normals_towards_estimated_center)
  {
    for (size_t i = 0; i < normals_in->size(); i++)
    {
      pcl::flipNormalTowardsViewpoint(cloud_in->points.at(i),
                                      estimated_center.x,
                                      estimated_center.y,
                                      estimated_center.z,
                                      normals_in->at(i).normal_x,
                                      normals_in->at(i).normal_y,
                                      normals_in->at(i).normal_z);
    }
  }

  return estimated_center;
}


template <typename PointT>
bool Superellipsoid<PointT>::fit(bool log_to_stdout, int max_num_iterations, CostFunctionType cost_type, double prior_center, double prior_scaling)
{
  parameters_ptr->resize(16);
  auto parameters = (*parameters_ptr).data();

  auto priors_ptr = std::make_shared<std::vector<double>>();
  priors_ptr->resize(16);
  auto priors = (*priors_ptr).data();

  // TODO: set start parameters via a function optionally
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
    CostFunction* cost_function = SuperellipsoidError::Create(point_.x(), point_.y(), point_.z(), priors, cost_type, prior_center, prior_scaling);
    //problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.2), parameters); // TODO: regularizer shouldn't be applied to prior residuals! Checking the ceres documentation may help.
    problem.AddResidualBlock(cost_function, nullptr, parameters); // TODO: add loss
  }

  // lower/upper bounds
  problem.SetParameterLowerBound(parameters, 0, 0.02); problem.SetParameterUpperBound(parameters, 0, 0.15); // a
  problem.SetParameterLowerBound(parameters, 1, 0.02); problem.SetParameterUpperBound(parameters, 1, 0.15); // b
  problem.SetParameterLowerBound(parameters, 2, 0.02); problem.SetParameterUpperBound(parameters, 2, 0.15); // c
  problem.SetParameterLowerBound(parameters, 3, 0.3); problem.SetParameterUpperBound(parameters, 3, 0.9); // e1
  problem.SetParameterLowerBound(parameters, 4, 0.3); problem.SetParameterUpperBound(parameters, 4, 0.9); // e2

  Solver::Options options;
  options.max_num_iterations = max_num_iterations;
  options.num_threads = 4; // can be good with SMT on modern CPU's
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.minimizer_progress_to_stdout = log_to_stdout;
  //options.check_gradients = true; // enabling this causing the optimization to be failed. weird.

  Solver::Summary summary;
  Solve(options, &problem, &summary);

  if (log_to_stdout)
  {
    std::cout << summary.BriefReport() << "\n";
    //std::cout << summary.FullReport() << "\n";
  }

  return summary.IsSolutionUsable(); // true if converged
}


template <typename PointT>
pcl::PointCloud<pcl::PointXYZ>::Ptr Superellipsoid<PointT>::estimateMissingSurfaces(double distance_threshold, int num_samples)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr fibonacci_superellipsoid = sampleSurface(true, true, num_samples);
  pcl::PointCloud<pcl::PointXYZ>::Ptr missing_surfaces = (new pcl::PointCloud<pcl::PointXYZ>)->makeShared();

  // TODO: For stability cloud_in can be projected onto the superellipsoid.
  //       May help in situations where points are noisy.
  pcl::search::KdTree<PointT> kdtree;
  kdtree.setInputCloud(cloud_in);

  for (size_t i = 0; i < fibonacci_superellipsoid->points.size(); i++)
  {
    auto p = fibonacci_superellipsoid->points.at(i);
    PointT searchPoint;
    searchPoint.x = p.x;
    searchPoint.y = p.y;
    searchPoint.z = p.z;

    int K = 1;
    std::vector<int> pointIdxKNNSearch(K);
    std::vector<float> pointKNNSquaredDistance(K);
    kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance);

    if ( pointKNNSquaredDistance.at(0) > pow(distance_threshold, 2) )
    {
      missing_surfaces->points.push_back(p);
    }
  }

  return missing_surfaces;
}


// See: https://salihmarangoz.github.io/blog/Superellipsoid_Sampling/
template <typename PointT>
pcl::PointCloud<pcl::PointXYZ>::Ptr Superellipsoid<PointT>::_sampleSurfaceFibonacciProjection(double a, double b, double c, double e1, double e2, double tx, double ty, double tz, double roll, double pitch, double yaw, int num_samples) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_pc = (new pcl::PointCloud<pcl::PointXYZ>)->makeShared ();
  double phi = M_PI * (3.0 - sqrt(5.0));  // golden angle in radians

  for (int i=0; i<num_samples; i++)
  {
    // Sample from fibonacci sphere
    double y = 1.0 - (i / (num_samples - 1.0)) * 2.0;  // y goes from 1 to -1
    double radius = sqrt(1 - y * y);  // radius at y
    double theta = phi * i;  // golden angle increment
    double x = cos(theta) * radius;
    double z = sin(theta) * radius;

    // Project the sampled point onto superellipsoid
    for (int j=0; j<10; j++) // 10 iterations are enough
    {
      // Compute distance to superellipsoid surface (approximation) and distance to center
      double f = pow(pow(abs(x),2.0/e2) + pow(abs(y),2.0/e2), e2/e1) + pow(abs(z),2.0/e1); // assuming a=1, b=1, c=1
      double distance_to_center = sqrt(pow(x,2) + pow(y,2) + pow(z,2));
      double distance_to_superellipsoid = std::copysign(abs(1.0-pow(f, e1/2.0)) , f-1); // negative if inside, positive if outside

      // Project using normalizer
      double normalizer = (distance_to_center - distance_to_superellipsoid) / distance_to_center;
      x = x * normalizer;
      y = y * normalizer;
      z = z * normalizer;
    }

    // scale with a,b,c
    x = x * a;
    y = y * b;
    z = z * c;

    // rotation and translation
    double angles[3] = {roll, pitch, yaw};
    double rotation_matrix_[9];
    ceres::EulerAnglesToRotationMatrix(angles, 3, rotation_matrix_);
    auto rotation_matrix = Eigen::Map<Eigen::Matrix<double,3,3> >(rotation_matrix_);
    Eigen::Matrix<double, 3, 1> point {x,y,z};
    auto rotated_point = rotation_matrix * point;
    pcl::PointXYZ transformed_point;
    transformed_point.x = rotated_point[0] + tx;
    transformed_point.y = rotated_point[1] + ty;
    transformed_point.z = rotated_point[2] + tz;
    output_pc->push_back(transformed_point);
  }

  return output_pc;
}

template <typename PointT>
pcl::PointCloud<pcl::PointXYZ>::Ptr Superellipsoid<PointT>::_sampleSurfaceParametricDefinition(double a, double b, double c, double e1, double e2, double tx, double ty, double tz, double roll, double pitch, double yaw, double u_res/*=0.05*/, double v_res/*=0.05*/) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_pc = (new pcl::PointCloud<pcl::PointXYZ>)->makeShared ();

  // rotation
  double angles[3] = {roll, pitch, yaw};
  double rotation_matrix[9];
  ceres::EulerAnglesToRotationMatrix(angles, 3, rotation_matrix);
  auto rotation_matrix_ = Eigen::Map<Eigen::Matrix<double,3,3> >(rotation_matrix);

  for (double uu=-M_PI; uu<M_PI; uu+=u_res)
  {
    for (double vv=-M_PI/2; vv<M_PI/2; vv+=v_res)
    {
      double r = 2./e2;
      double t = 2./e1;

      double x = a * c_func(vv, 2./t) * c_func(uu, 2./r);
      double y = b * c_func(vv, 2./t) * s_func(uu, 2./r);
      double z = c * s_func(vv, 2./t);

      Eigen::Matrix<double, 3, 1> point {x,y,z};
      auto point_ = rotation_matrix_ * point;
      pcl::PointXYZ point__;
      point__.x = point_[0] + tx;
      point__.y = point_[1] + ty;
      point__.z = point_[2] + tz;
      output_pc->push_back(point__);
    }
  }

  return output_pc;
}


// See: https://en.wikipedia.org/wiki/Superellipsoid
template <typename PointT>
pcl::PointCloud<pcl::PointXYZ>::Ptr Superellipsoid<PointT>::sampleSurface(bool use_fibonacci_projection/*=false*/, bool apply_transformation/*=true*/, int num_samples_fibonacci/*=1000*/, double u_res_parametric/*=0.05*/, double v_res_parametric/*0.05*/) const
{
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

  if (!apply_transformation)
  {
    tx_ = 0.0;
    ty_ = 0.0;
    tz_ = 0.0;
    roll_ = 0.0;
    pitch_ = 0.0;
    yaw_ = 0.0;
  }

  if (use_fibonacci_projection)
  {
    return _sampleSurfaceFibonacciProjection(a_, b_, c_, e1_, e2_, tx_, ty_, tz_, roll_, pitch_, yaw_, num_samples_fibonacci);
  }
  else
  {
    return _sampleSurfaceParametricDefinition(a_, b_, c_, e1_, e2_, tx_, ty_, tz_, roll_, pitch_, yaw_, u_res_parametric, v_res_parametric);
  }
}


// See: https://en.wikipedia.org/wiki/Superellipsoid
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
pcl::PointXYZ Superellipsoid<PointT>::getEstimatedCenter() const
{
  return estimated_center;
}


// void::setEstimatedCenter()
// {
//   return estimated_center;
// }


template <typename PointT>
const typename pcl::PointCloud<PointT>::Ptr Superellipsoid<PointT>::getCloud() const
{
  return cloud_in;
}


template <typename PointT>
const pcl::PointCloud<pcl::Normal>::Ptr Superellipsoid<PointT>::getNormals() const
{
  return normals_in;
}


// TODO
/*
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> separateCloudByIndices(const typename pcl::PointCloud<PointT>::ConstPtr &input_cloud, const pcl::IndicesConstPtr &indices)
{
  typename pcl::PointCloud<PointT>::Ptr inlier_cloud(new pcl::PointCloud<PointT>), outlier_cloud(new pcl::PointCloud<PointT>);
  inlier_cloud->header = input_cloud->header;
  outlier_cloud->header = input_cloud->header;
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(input_cloud);
  extract.setIndices(indices);
  extract.setNegative(false);
  extract.filter(*inlier_cloud);
  extract.setNegative(true);
  extract.filter(*outlier_cloud);
  return std::make_pair(inlier_cloud, outlier_cloud);
}
*/

} // namespace superellipsoid
#endif // SUPERELLIPSOID_H

