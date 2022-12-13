#include <ros/ros.h>
#include <capsicum_superellipsoid_detector/superellipsoid_detector.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "capsicum_superellipsoid_detector_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");
  superellipsoid::SuperellipsoidDetector sd(nh, priv_nh);
  sd.startNode();
  ros::spin();
  return 0;
}