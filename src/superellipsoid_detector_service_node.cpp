#include <ros/ros.h>
#include <capsicum_superellipsoid_detector/superellipsoid_detector_service.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "capsicum_superellipsoid_detector_service");
  superellipsoid::SuperellipsoidDetectorService dss;
  dss.init();
  ros::spin();
  return 0;
}