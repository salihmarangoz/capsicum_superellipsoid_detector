#ifndef __SUPERELLIPSOID_ROS_CONVERSION_H__
#define __SUPERELLIPSOID_ROS_CONVERSION_H__

#include <capsicum_superellipsoid_detector/superellipsoid.h>
#include <superellipsoid_msgs/Superellipsoid.h>
#include <superellipsoid_msgs/SuperellipsoidArray.h>

namespace superellipsoid_msgs
{
//////////////////////////////////////////////////////////////////
//////// SUPERELLIPSOID CAPSULATED TYPES /////////////////////////
//////////////////////////////////////////////////////////////////

// Superellipsoid -> encapsulated by a pointer
using SuperellipsoidPtr = boost::shared_ptr<Superellipsoid>;

// Superellipsoid -> inside a vector -> encapsulated by a pointer
using SuperellipsoidArrayPtr = boost::shared_ptr<SuperellipsoidArray>;


template <typename PointT>
superellipsoid::Superellipsoid<PointT> fromROSMsg(const superellipsoid_msgs::Superellipsoid &se_msg)
{
  superellipsoid::Superellipsoid<PointT> se;
  (*(se.parameters_ptr))[0] = se_msg.a;
  (*(se.parameters_ptr))[1] = se_msg.b;
  (*(se.parameters_ptr))[2] = se_msg.c;
  (*(se.parameters_ptr))[3] = se_msg.e1;
  (*(se.parameters_ptr))[4] = se_msg.e2;
  (*(se.parameters_ptr))[5] = se_msg.tx;
  (*(se.parameters_ptr))[6] = se_msg.ty;
  (*(se.parameters_ptr))[7] = se_msg.tz;
  (*(se.parameters_ptr))[8] = se_msg.roll;
  (*(se.parameters_ptr))[9] = se_msg.pitch;
  (*(se.parameters_ptr))[10] = se_msg.yaw;
  return se;
}

template <typename PointT>
superellipsoid::SuperellipsoidArray<PointT> fromROSMsg(const superellipsoid_msgs::SuperellipsoidArray &se_msg)
{
  superellipsoid::SuperellipsoidArray<PointT> arr;

  for (int i=0; i<se_msg.superellipsoids.size(); i++)
  {
    superellipsoid::Superellipsoid<PointT> se = fromROSMsg<PointT>(se_msg.superellipsoids[i]);
    arr.push_back(se);
  }

  return arr;
}

template <typename PointT>
superellipsoid_msgs::Superellipsoid toROSMsg(const superellipsoid::Superellipsoid<PointT> &se, const std_msgs::Header &header)
{
  superellipsoid_msgs::Superellipsoid se_msg;
  se_msg.a = (*(se.parameters_ptr))[0];
  se_msg.b = (*(se.parameters_ptr))[1];
  se_msg.c = (*(se.parameters_ptr))[2];
  se_msg.e1 = (*(se.parameters_ptr))[3];
  se_msg.e2 = (*(se.parameters_ptr))[4];
  se_msg.tx = (*(se.parameters_ptr))[5];
  se_msg.ty = (*(se.parameters_ptr))[6];
  se_msg.tz = (*(se.parameters_ptr))[7];
  se_msg.roll = (*(se.parameters_ptr))[8];
  se_msg.pitch = (*(se.parameters_ptr))[9];
  se_msg.yaw = (*(se.parameters_ptr))[10];
  se_msg.volume = se.computeVolume();
  se_msg.header = header;
  return se_msg;
}

template <typename PointT>
superellipsoid_msgs::SuperellipsoidArray toROSMsg(const superellipsoid::SuperellipsoidArray<PointT> &se, const std_msgs::Header &header)
{
  superellipsoid_msgs::SuperellipsoidArray se_msg_arr;

  se_msg_arr.superellipsoids.reserve(se.size());
  for (int i=0; i<se.size(); i++)
  {
    superellipsoid_msgs::Superellipsoid se_msg = toROSMsg(se[i], header);
    se_msg_arr.superellipsoids.push_back(se_msg);
  }

  se_msg_arr.header = header;
  return se_msg_arr;
}

//extra:
template <typename PointT>
superellipsoid_msgs::SuperellipsoidArray toROSMsg(const std::vector<superellipsoid_msgs::Superellipsoid> &se_msg_vec, const std_msgs::Header &header, bool overwrite_headers=false)
{
  superellipsoid_msgs::SuperellipsoidArray se_msg_arr;
  se_msg_arr.superellipsoids = se_msg_vec; // copy all superellipsoids
  se_msg_arr.header = header;

  if (overwrite_headers)
  {
    for (int i=0; i<se_msg_arr.superellipsoids.size(); i++)
    {
      se_msg_arr.superellipsoids[i].header = header;
    }
  }

  return se_msg_arr;
}

//TODO: TF2 export



} // namespace superellipsoid_msgs
#endif // __SUPERELLIPSOID_ROS_CONVERSION_H__