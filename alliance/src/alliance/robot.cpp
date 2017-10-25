#include "alliance/robot.h"

namespace alliance
{
Robot::Robot(const std::string& id, const std::string& name,
             const std::string& ns)
    : RobotInterface<BehaviourSet>::RobotInterface(id, name, ns),
      broadcast_rate_(0.0), timeout_duration_(0.0)
{
}

Robot::~Robot() {}

ros::Rate Robot::getBroadcastRate() const { return broadcast_rate_; }

ros::Duration Robot::getTimeoutDuration() const { return timeout_duration_; }

void Robot::setBroadcastRate(const ros::Rate& broadcast_rate)
{
  broadcast_rate_ = broadcast_rate;
}

void Robot::setTimeoutDuration(const ros::Duration& timeout_duration)
{
  timeout_duration_ = timeout_duration;
}

void Robot::addBehaviourSet(const BehaviourSetPtr& behaviour_set)
{
  int size(behaviour_sets_.size());
  RobotInterface<BehaviourSet>::addBehaviourSet(behaviour_set);
  if (size == behaviour_sets_.size())
  {
    return;
  }
  for (iterator it(behaviour_sets_.begin()); it != behaviour_sets_.end(); it++)
  {
    BehaviourSetPtr robot_behaviour_set(*it);
    behaviour_set->registerActivitySuppression(robot_behaviour_set);
    robot_behaviour_set->registerActivitySuppression(behaviour_set);
  }
}
}
