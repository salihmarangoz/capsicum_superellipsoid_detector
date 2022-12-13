#ifndef __SUPERELLIPSOID_DETECTOR_H__
#define __SUPERELLIPSOID_DETECTOR_H__

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <capsicum_superellipsoid_detector/SuperellipsoidDetectorConfig.h>

namespace superellipsoid
{

class SuperellipsoidDetector
{
public:
    SuperellipsoidDetector(ros::NodeHandle &nh, ros::NodeHandle &priv_nh);
    ~SuperellipsoidDetector();
    void initNode();
    void initService();
    
private:
    void configCallback(capsicum_superellipsoid_detector::SuperellipsoidDetectorConfig &config, uint32_t level);

    ros::NodeHandle &m_nh;
    ros::NodeHandle &m_priv_nh;
    capsicum_superellipsoid_detector::SuperellipsoidDetectorConfig m_config;
    dynamic_reconfigure::Server<capsicum_superellipsoid_detector::SuperellipsoidDetectorConfig> m_reconfigure_server;
};



} // namespace superellipsoid

#endif // __SUPERELLIPSOID_DETECTOR_H__