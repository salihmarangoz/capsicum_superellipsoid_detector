

#ifndef SUPERELLIPSOID_H
#define SUPERELLIPSOID_H

#include <memory>
#include <vector>
#include <string>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>

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


class Superellipsoid
{
public:
  Superellipsoid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
  pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(float search_radius);
  pcl::PointXYZ estimateClusterCenter(float regularization);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sampleSurface();
  pcl::PointCloud<pcl::PointXYZ>::Ptr sampleVolume(double resolution);
  pcl::PointXYZ getOptimizedCenter();
  pcl::PointXYZ getEstimatedCenter();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud();
  pcl::PointCloud<pcl::Normal>::Ptr getNormals();

  bool fit(bool log_to_stdout);
  std::map<std::string, double> getParameters();

  double c_func(double w, double m);
  double s_func(double w, double m);

  //void generateSuperellipsoidVolume();

private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in;
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


} // namespace
#endif // SUPERELLIPSOID_H




/*  TEST SUPERELLIPSOID
    auto parameters = (*parameters_ptr).data();
    parameters[0] = 0.03; // a
    parameters[1] = 0.03; // b
    parameters[2] = 0.03; // c
    parameters[3] = 0.6; // e1
    parameters[4] = 0.6; // e2
    parameters[5] = 1.0;
    parameters[6] = 0.25;
    parameters[7] = 0.5;
    parameters[8] = 60.0;  // roll
    parameters[9] = 0.0; // pitch
    parameters[10] = 30.0; // yaw
*/

/* ---- TODO -----

*/