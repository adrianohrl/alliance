#include "alliance/acquiescence.h"
#include "alliance/robot.h"

namespace alliance
{
Acquiescence::Acquiescence(const RobotPtr& robot,
                           const BehaviourSetPtr& behaviour_set)
    : robot_(robot), behaviour_set_(behaviour_set),
      yielding_delay_(new SampleHolder(
          behaviour_set_->getId() + "/acquiescence/yielding_delay", 0.0,
          behaviour_set_->getBufferHorizon())),
      giving_up_delay_(new SampleHolder(
          behaviour_set_->getId() + "/acquiescence/giving_up_delay", 0.0,
          behaviour_set_->getBufferHorizon()))
{
}

Acquiescence::~Acquiescence() {}

void Acquiescence::init(const InterRobotCommunicationPtr& monitor)
{
  if (!monitor_)
  {
    monitor_ = monitor;
  }
}

ros::Duration Acquiescence::getYieldingDelay(const ros::Time& timestamp) const
{
  return ros::Duration(yielding_delay_->getValue(timestamp));
}

ros::Duration Acquiescence::getGivingUpDelay(const ros::Time& timestamp) const
{
  return ros::Duration(giving_up_delay_->getValue(timestamp));
}

bool Acquiescence::isAcquiescent(const ros::Time& timestamp)
{
  ros::Time activation_timestamp(behaviour_set_->getActivationTimestamp());
  if (activation_timestamp.isZero() || timestamp < activation_timestamp)
  {
    return false;
  }
  double elapsed_duration((timestamp - activation_timestamp).toSec());
  return (elapsed_duration > yielding_delay_->getValue(timestamp) &&
          monitor_->received(timestamp - robot_->getTimeoutDuration(),
                             timestamp)) ||
         elapsed_duration > giving_up_delay_->getValue(timestamp);
}

void Acquiescence::setYieldingDelay(const ros::Duration& yielding_delay,
                                    const ros::Time& timestamp)
{
  ROS_DEBUG_STREAM("Updating " << *yielding_delay_ << " to "
                               << yielding_delay.toSec() << " [s].");
  yielding_delay_->update(yielding_delay.toSec(), timestamp);
}

void Acquiescence::setGivingUpDelay(const ros::Duration& giving_up_delay,
                                    const ros::Time& timestamp)
{
  ROS_DEBUG_STREAM("Updating " << *giving_up_delay_ << " to "
                               << giving_up_delay.toSec() << " [s].");
  giving_up_delay_->update(giving_up_delay.toSec(), timestamp);
}
}
