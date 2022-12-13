#include <ros/ros.h>
#include <capsicum_superellipsoid_detector/superellipsoid_detector.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "capsicum_superellipsoid_detector_node");
  superellipsoid::SuperellipsoidDetector sd;
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");
  sd.init(nh, priv_nh);
  ros::spin();
  return 0;
}