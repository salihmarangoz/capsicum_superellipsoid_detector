#ifndef __SUPERELLIPSOID_UTILS_H__
#define __SUPERELLIPSOID_UTILS_H__

#include <pcl/point_cloud.h>

template<typename PointIn, typename PointWithLabelOut>
void labelPointCloud(const pcl::PointCloud<PointIn> &pc_in, pcl::PointCloud<PointWithLabelOut> &pc_with_label_out, uint32_t label)
{
  pcl::PointCloud<PointWithLabelOut> pc_out;
  pcl::copyPointCloud(pc_in, pc_with_label_out);
  for (int i=0; i<pc_with_label_out.size(); i++)
  {
    pc_with_label_out[i].label = label;
  }
}

template<typename PointIn, typename PointWithLabelOut>
pcl::PointCloud<PointWithLabelOut> labelPointCloud(const pcl::PointCloud<PointIn> &pc_in, uint32_t label)
{
  pcl::PointCloud<PointWithLabelOut> pc_with_label_out;
  labelPointCloud<PointIn, PointWithLabelOut>(pc_in, pc_with_label_out, label);
  return pc_with_label_out;
}

template<typename PointIn, typename PointWithLabelOut>
void labelPoint(const PointIn p_in, PointWithLabelOut &p_with_label_out, uint32_t label)
{
  pcl::copyPoint(p_in, p_with_label_out);
  p_with_label_out.label = label;
}

template<typename PointIn, typename PointWithLabelOut>
PointWithLabelOut labelPoint(const PointIn p_in, uint32_t label)
{
  PointWithLabelOut p_with_label_out;
  labelPoint<PointIn, PointWithLabelOut>(p_in, p_with_label_out, label);
  return p_with_label_out;
}


#endif // __SUPERELLIPSOID_UTILS_H__