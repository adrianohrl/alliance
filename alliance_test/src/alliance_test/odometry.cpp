#include "alliance_test/odometry.h"
#include <tf/transform_datatypes.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(alliance_test::Odometry, alliance::Sensor)

namespace alliance_test
{
Odometry::Odometry() {}

Odometry::~Odometry() {}

double Odometry::getX() const { return msg_.pose.pose.position.x; }

double Odometry::getY() const { return msg_.pose.pose.position.y; }

double Odometry::getYaw() const
{
  double yaw(tf::getYaw(msg_.pose.pose.orientation));
  while (yaw <= -M_PI || yaw > M_PI)
  {
    yaw += (yaw > M_PI ? -1 : 1) * 2 * M_PI;
  }
  return yaw;
}
}
