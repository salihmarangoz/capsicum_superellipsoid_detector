#include <pluginlib/class_list_macros.h>
#include <capsicum_superellipsoid_detector/superellipsoid_detector_nodelet.hpp>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(capsicum_superellipsoid_detector::SuperellipsoidDetectorNodelet, nodelet::Nodelet)

namespace superellipsoid
{

void SuperellipsoidDetectorNodelet::onInit()
{
    ds.init(getNodeHandle(), getPrivateNodeHandle());
}

} // namespace superellipsoid