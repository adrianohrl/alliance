#include "alliance/impatience.h"
#include "alliance/robot.h"

namespace alliance
{
Impatience::Impatience(const RobotPtr& robot,
                       const BehaviourSetPtr& behaviour_set)
    : AllianceObserver<alliance_msgs::InterRobotCommunication>::
          AllianceObserver(behaviour_set->getId() + "/impatience"),
      robot_(robot), behaviour_set_(behaviour_set),
      fast_rate_(new SampleHolder(id_ + "/fast_rate", 0.0,
                                  behaviour_set_->getBufferHorizon()))
{
}

Impatience::~Impatience() {}

void Impatience::init(const InterRobotCommunicationPtr& monitor)
{
  if (!monitor_)
  {
    monitor_ = monitor;
  }
}

double Impatience::getSlowRate(const std::string& robot_id,
                               const ros::Time& timestamp) const
{
  const_iterator it(slow_rates_.find(robot_id));
  if (it != slow_rates_.end())
  {
    SampleHolderPtr sample_holder(it->second);
    return sample_holder->getValue(timestamp);
  }
  return 0.0;
}

double Impatience::getFastRate(const ros::Time& timestamp) const
{
  return fast_rate_->getValue(timestamp);
}

ros::Duration
Impatience::getReliabilityDuration(const std::string& robot_id,
                                   const ros::Time& timestamp) const
{
  const_iterator it(reliability_durations_.find(robot_id));
  if (it != reliability_durations_.end())
  {
    SampleHolderPtr sample_holder(it->second);
    return ros::Duration(sample_holder->getValue(timestamp));
  }
  return ros::Duration();
}

double Impatience::getLevel(const ros::Time& timestamp) const
{
  double level(fast_rate_->getValue(timestamp));
  SampleHolderPtr sample_holder;
  const_iterator slow_rates_it(slow_rates_.begin()),
      reliability_durations_it(reliability_durations_.begin());
  while (slow_rates_it != slow_rates_.end())
  {
    sample_holder = reliability_durations_it->second;
    if (monitor_->received(slow_rates_it->first,
                           timestamp - robot_->getTimeoutDuration(),
                           timestamp) &&
        !monitor_->received(
            slow_rates_it->first, ros::Time(),
            timestamp - ros::Duration(sample_holder->getValue(timestamp))))
    {
      sample_holder = slow_rates_it->second;
      if (sample_holder->getValue(timestamp) < level)
      {
        level = sample_holder->getValue(timestamp);
      }
    }
    slow_rates_it++;
    reliability_durations_it++;
  }
  return level;
}

void Impatience::setSlowRate(const std::string& robot_id, double slow_rate,
                             const ros::Time& timestamp)
{
  if (slow_rate <= 0.0)
  {
    throw utilities::Exception("The " + id_ + " impatience slow rate must be positive.");
  }
  if (slow_rate >= fast_rate_->getValue(timestamp))
  {
    throw utilities::Exception(
        "The " + id_ + " slow rate must be greater than the fast one.");
  }
  SampleHolderPtr sample_holder;
  iterator it(slow_rates_.find(robot_id));
  if (it == slow_rates_.end())
  {
    sample_holder.reset(
        new SampleHolder(robot_->getId() + robot_id + "/slow_rate",
                         slow_rate, robot_->getTimeoutDuration(), timestamp));
    slow_rates_[robot_id] = sample_holder;
  }
  else
  {
    sample_holder = it->second;
  }
  ROS_DEBUG_STREAM("Updating " << *sample_holder << " to " << slow_rate << ".");
  sample_holder->update(slow_rate, timestamp);
}

void Impatience::setFastRate(double fast_rate, const ros::Time& timestamp)
{
  if (fast_rate <= 0.0)
  {
    throw utilities::Exception("The impatience fast rate must be positive.");
  }
  ROS_DEBUG_STREAM("Updating " << *fast_rate_ << " to " << fast_rate << ".");
  fast_rate_->update(fast_rate, timestamp);
}

void Impatience::setReliabilityDuration(
    const std::string& robot_id, const ros::Duration& reliability_duration,
    const ros::Time& timestamp)
{
  SampleHolderPtr sample_holder;
  iterator it(reliability_durations_.find(robot_id));
  if (it == reliability_durations_.end())
  {
    sample_holder.reset(new SampleHolder(
        robot_->getId() + robot_id + "/reliability_duration",
        reliability_duration.toSec(), robot_->getTimeoutDuration(), timestamp));
    reliability_durations_[robot_id] = sample_holder;
  }
  else
  {
    sample_holder = it->second;
  }
  ROS_DEBUG_STREAM("Updating " << *sample_holder << " to "
                               << reliability_duration.toSec() << " [s].");
  sample_holder->update(reliability_duration.toSec(), timestamp);
}

void Impatience::update(
    const nodes::InterRobotCommunicationEventConstPtr& event)
{
  if (event->isRelated(*robot_) ||
      !event->isRelated(*behaviour_set_->getTask()))
  {
    return;
  }
  std::string robot_id(event->getMsg().header.frame_id);
  ros::Time timestamp(event->getTimestamp());
  ros::Duration reliability_duration(event->getMsg().expected_duration);
  if (reliability_duration == getReliabilityDuration(robot_id, timestamp))
  {
    return;
  }
  double slow_rate(
      behaviour_set_->getMotivationalBehaviour()->getThreshold(timestamp) /
      reliability_duration.toSec());
  setReliabilityDuration(robot_id, reliability_duration, timestamp);
  setSlowRate(robot_id, slow_rate, timestamp);
}
}
