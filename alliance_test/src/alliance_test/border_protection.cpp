#include "alliance_test/border_protection.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(alliance_test::BorderProtection, alliance::Layer)

namespace alliance_test
{
BorderProtection::BorderProtection()
    : align_left_(false), align_right_(false), x_min_(0.0), y_min_(0.0),
      x_max_(0.0), y_max_(0.0)
{
}

BorderProtection::~BorderProtection() {}

void BorderProtection::process()
{
  Layer::process();
  /*double vx(0.0), wz(0.0);
  ROS_INFO_STREAM("[BorderProtection] x: " << odometry_->getX() << ", y: " << odometry_->getY());
  if (odometry_->getX() >= x_min_ && odometry_->getX() <= x_max_ &&
      odometry_->getY() >= y_min_ && odometry_->getY() <= y_max_)
  {
    vx =
        GAIN * (std::max(sonars_->getDistance(2), sonars_->getDistance(3)) -
                1 / std::max(sonars_->getDistance(2), sonars_->getDistance(3)));
    wz = 0.5 * GAIN /
         std::min(sonars_->getDistance(2), sonars_->getDistance(3)) *
         (std::min(sonars_->getDistance(0),
                   std::min(sonars_->getDistance(1), sonars_->getDistance(2))) -
          std::min(sonars_->getDistance(3),
                   std::min(sonars_->getDistance(4), sonars_->getDistance(5))));
  }
  else if (sonars_->getDistance(0) < sonars_->getDistance(5) && align_left_)
  {
    align_right_ = false;
    vx = 0.5;
    wz = 0.4 * (sonars_->getDistance(0) - DANGEROUS_DISTANCE) +
         1.2 * (sonars_->getDistance(1) - 0.75 * sonars_->getDistance(7)) +
         0.3 * (sonars_->getDistance(3) -
                std::max(sonars_->getDistance(4),
                         std::max(sonars_->getDistance(6),
                                  sonars_->getDistance(5))));
  }
  else if (sonars_->getDistance(5) < sonars_->getDistance(0) && align_right_)
  {
    align_left_ = false;
    vx = 0.5;
    wz = -0.4 * (sonars_->getDistance(5) - DANGEROUS_DISTANCE) -
         1.2 * (sonars_->getDistance(4) - 0.75 * sonars_->getDistance(6)) -
         0.3 * (sonars_->getDistance(2) -
                std::max(sonars_->getDistance(1),
                         std::max(sonars_->getDistance(7),
                                  sonars_->getDistance(0))));
  }
  else
  {
    align_left_ = true;
  }
  Layer::setVelocity(vx, wz);*/
  Layer::setVelocity(0.0, -0.5);
}

void BorderProtection::initialize(const std::string& ns,
                                  const std::string& name)
{
  Layer::initialize(ns, name);
  ros::NodeHandle pnh("~/BorderProtection");
  pnh.param("min_x", x_min_, 0.0);
  pnh.param("min_y", y_min_, 0.0);
  pnh.param("max_x", x_max_, 0.0);
  pnh.param("max_y", y_max_, 0.0);
}
}
