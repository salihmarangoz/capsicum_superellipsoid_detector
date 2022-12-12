#include <ros/ros.h>
#include "capsicum_superellipsoid_detector/superellipsoid_detector.h">

int main(int argc, char** argv)
{
  ros::init(argc, argv, "capsicum_superellipsoid_detector");
  SuperellipsoidDetector sed;
  ros::spin();
  return 0;
}