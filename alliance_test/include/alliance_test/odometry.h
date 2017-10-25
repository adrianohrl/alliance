#ifndef _SENSORS_ODOMETRY_H_
#define _SENSORS_ODOMETRY_H_

#include <nav_msgs/Odometry.h>
#include <nodes/ros_sensor_message.h>

namespace alliance_test
{
class Odometry : public nodes::ROSSensorMessage<nav_msgs::Odometry>
{
public:
  Odometry();
  virtual ~Odometry();
  double getX() const;
  double getY() const;
  double getYaw() const;
};

typedef boost::shared_ptr<Odometry> OdometryPtr;
typedef boost::shared_ptr<Odometry const> OdometryConstPtr;
}

#endif // _SENSORS_ODOMETRY_H_
