#include "alliance_test/wander.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(alliance_test::Wander, alliance::Layer)

namespace alliance_test
{
Wander::Wander() {}

Wander::~Wander() {}

void Wander::process()
{
  Layer::process();
  if (!sonars_->isUpToDate())
  {
    ROS_INFO("stopping!!!");
    Layer::setVelocity();
    return;
  }
  double vx(-10.0), wz(10.0);
  if (isSafe())
  {
    vx = 0.25 * GAIN *
         (std::max(sonars_->getDistance(2), sonars_->getDistance(3)) -
          1 / std::max(sonars_->getDistance(2), sonars_->getDistance(3)));
    wz = 0.1 * GAIN /
         std::max(sonars_->getDistance(2), sonars_->getDistance(3)) *
         (std::min(sonars_->getDistance(0),
                   std::min(sonars_->getDistance(1), sonars_->getDistance(2))) -
          std::min(sonars_->getDistance(3),
                   std::min(sonars_->getDistance(4), sonars_->getDistance(5))));
  }
  else if (!isInDanger())
  {
    vx = 0.1 * GAIN *
         (std::max(sonars_->getDistance(2), sonars_->getDistance(3)) -
          1 / std::max(sonars_->getDistance(2), sonars_->getDistance(3)));
    wz = 0.5 * GAIN /
         std::min(sonars_->getDistance(2), sonars_->getDistance(3)) *
         (std::min(sonars_->getDistance(0),
                   std::min(sonars_->getDistance(1), sonars_->getDistance(2))) -
          std::min(sonars_->getDistance(3),
                   std::min(sonars_->getDistance(4), sonars_->getDistance(5))));
  }
  Layer::setVelocity(vx, wz);
}

bool Wander::isSafe() const
{
  return sonars_->getDistance(0) > SAFE_DISTANCE &&
         sonars_->getDistance(1) > SAFE_DISTANCE &&
         sonars_->getDistance(2) > SAFE_DISTANCE &&
         sonars_->getDistance(3) > SAFE_DISTANCE &&
         sonars_->getDistance(4) > SAFE_DISTANCE &&
         sonars_->getDistance(5) > SAFE_DISTANCE;
}

bool Wander::isInDanger() const
{
  return fabs(std::min(
                  sonars_->getDistance(0),
                  std::min(sonars_->getDistance(1), sonars_->getDistance(2))) -
              std::min(sonars_->getDistance(3),
                       std::min(sonars_->getDistance(4),
                                sonars_->getDistance(5)))) < TOLERANCE &&
         fabs(std::min(
                  sonars_->getDistance(0),
                  std::min(sonars_->getDistance(1), sonars_->getDistance(2))) -
              std::min(sonars_->getDistance(3),
                       std::min(sonars_->getDistance(4),
                                sonars_->getDistance(5)))) < TOLERANCE;
}
}
