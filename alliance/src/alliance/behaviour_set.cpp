#include "alliance/behaviour_set.h"
#include "alliance/robot.h"
#include <utilities/toggle_event.h>

namespace alliance
{
BehaviourSet::BehaviourSet(const RobotPtr& robot, const TaskPtr& task,
                           const ros::Duration& buffer_horizon)
    : BehaviourSetInterface<Robot>::BehaviourSetInterface(robot, task,
                                                          buffer_horizon),
      Subject::Subject(ns_), robot_(robot)
{
}

BehaviourSet::~BehaviourSet() {}

void BehaviourSet::init()
{
  if (!motivational_behaviour_)
  {
    motivational_behaviour_.reset(
        new MotivationalBehaviour(robot_, shared_from_this()));
    motivational_behaviour_->init();
    ROS_DEBUG_STREAM(*this << " has been initialized.");
  }
}

void BehaviourSet::preProcess()
{
  ros::Time timestamp(ros::Time::now());
  setActive(motivational_behaviour_->isActive(timestamp), timestamp);
}

MotivationalBehaviourPtr BehaviourSet::getMotivationalBehaviour() const
{
  return motivational_behaviour_;
}

void BehaviourSet::setActivationThreshold(double threshold)
{
  if (!motivational_behaviour_)
  {
    throw utilities::Exception("The motivational behaviour of the " + str() +
                               " behaviour set has not been initialized yet.");
  }
  motivational_behaviour_->setThreshold(threshold);
}

void BehaviourSet::setAcquiescence(const ros::Duration& yielding_delay,
                                   const ros::Duration& giving_up_delay)
{
  motivational_behaviour_->setAcquiescence(yielding_delay, giving_up_delay);
}

void BehaviourSet::setImpatience(double fast_rate)
{
  motivational_behaviour_->setImpatience(fast_rate);
}

void BehaviourSet::registerActivitySuppression(
    const BehaviourSetPtr& behaviour_set)
{
  if (*this != *behaviour_set)
  {
    Subject::registerObserver(
        behaviour_set->motivational_behaviour_->getActivitySuppression());
  }
}

void BehaviourSet::setActive(bool active, const ros::Time& timestamp)
{
  if (active != isActive(timestamp))
  {
    BehaviourSetInterface<Robot>::setActive(active, timestamp);
    utilities::ToggleEventConstPtr event(
        new utilities::ToggleEvent(shared_from_this(), active, timestamp));
    Subject::notify(event);
  }
}
}
