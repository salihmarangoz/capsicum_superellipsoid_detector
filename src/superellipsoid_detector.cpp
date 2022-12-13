#include <capsicum_superellipsoid_detector/superellipsoid_detector.h>

namespace superellipsoid
{


SuperellipsoidDetector::SuperellipsoidDetector(ros::NodeHandle &nh, ros::NodeHandle &priv_nh) : m_nh(nh), m_priv_nh(priv_nh)
{

}

SuperellipsoidDetector::~SuperellipsoidDetector()
{
    
}

SuperellipsoidDetector::initNode()
{
    
}

SuperellipsoidDetector::initService()
{
    
}

void config_callback(capsicum_superellipsoid_detector::SuperellipsoidDetectorConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
            config.int_param, config.double_param, 
            config.str_param.c_str(), 
            config.bool_param?"True":"False", 
            config.size);
}

} // namespace superellipsoid
