#include "alliance_test/point_cloud.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(alliance_test::PointCloud, alliance::Sensor)

namespace alliance_test
{
PointCloud::PointCloud() {}

PointCloud::~PointCloud() {}

double PointCloud::operator[](int index) const
{
  return index >= 0 && index < msg_.points.size()
             ? sqrt(pow(msg_.points[index].x, 2) + pow(msg_.points[index].y, 2))
             : 0.0;
}

double PointCloud::getDistance(int index) const
{
  return index >= 0 && index < msg_.points.size()
             ? sqrt(pow(msg_.points[index].x, 2) + pow(msg_.points[index].y, 2))
             : 0.0;
}

geometry_msgs::Point32 PointCloud::getPoint(int index) const
{
  return msg_.points[index];
}

bool PointCloud::empty() const { return msg_.points.empty(); }

std::size_t PointCloud::size() const { return msg_.points.size(); }

PointCloud::iterator PointCloud::begin() { return msg_.points.begin(); }

PointCloud::const_iterator PointCloud::begin() const { return msg_.points.begin(); }

PointCloud::iterator PointCloud::end() { return msg_.points.end(); }

PointCloud::const_iterator PointCloud::end() const { return msg_.points.end(); }
}
