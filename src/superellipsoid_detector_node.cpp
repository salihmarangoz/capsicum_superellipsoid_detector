#include <ros/ros.h>
#include "capsicum_superellipsoid_detector/superellipsoid_detector_server.h">

int main(int argc, char** argv)
{
  ros::init(argc, argv, "capsicum_superellipsoid_detector");
  SuperellipsoidDetectorServer ds;
  ros::spin();
  return 0;
}