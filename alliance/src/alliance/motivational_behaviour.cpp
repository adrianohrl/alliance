#include "alliance/motivational_behaviour.h"
#include "alliance/behaviour_set.h"
#include "alliance/robot.h"

namespace alliance
{
MotivationalBehaviour::MotivationalBehaviour(
    const RobotPtr& robot, const BehaviourSetPtr& behaviour_set)
    : robot_(robot), behaviour_set_(behaviour_set),
      threshold_(new SampleHolder(behaviour_set->getId() + "/threshold", 0.0,
                                  behaviour_set_->getBufferHorizon())),
      motivation_(new SampleHolder(behaviour_set->getId() + "/motivation", 0.0,
                                   behaviour_set_->getBufferHorizon())),
      monitor_(new InterRobotCommunication(robot, behaviour_set)),
      acquiescence_(new Acquiescence(robot, behaviour_set)),
      activity_suppression_(new ActivitySuppression(robot, behaviour_set)),
      impatience_(new Impatience(robot, behaviour_set)),
      impatience_reset_(new ImpatienceReset()),
      sensory_feedback_(new SensoryFeedback(robot, behaviour_set))
{
  msg_.header.frame_id = robot_->getId();
  msg_.task_id = behaviour_set_->getTask()->getId();
  msg_.motivation = 0.0;
}

MotivationalBehaviour::~MotivationalBehaviour() {}

void MotivationalBehaviour::init()
{
  acquiescence_->init(monitor_);
  impatience_->init(monitor_);
  impatience_reset_->init(monitor_);
}

bool MotivationalBehaviour::isActive(const ros::Time& timestamp)
{
  msg_.threshold = getThreshold(timestamp);
  msg_.active = getLevel(timestamp) >= msg_.threshold;
  if (msg_.active)
  {
    msg_.motivation = msg_.threshold;
  }
  return msg_.active;
}

double MotivationalBehaviour::getThreshold(const ros::Time& timestamp) const
{
  return threshold_->getValue(timestamp);
}

double MotivationalBehaviour::getLevel(const ros::Time& timestamp)
{
  msg_.header.stamp = timestamp;
  msg_.impatience = impatience_->getLevel(timestamp);
  msg_.acquiescent = acquiescence_->isAcquiescent(timestamp);
  msg_.suppressed = activity_suppression_->isSuppressed(timestamp);
  msg_.resetted = impatience_reset_->isResetted(timestamp);
  msg_.aplicable = sensory_feedback_->isApplicable(timestamp);
  msg_.motivation = (msg_.motivation + msg_.impatience) * !msg_.acquiescent *
                    !msg_.suppressed * !msg_.resetted * msg_.aplicable;
  motivation_->update(msg_.motivation, timestamp);
  return msg_.motivation;
}

ActivitySuppressionPtr MotivationalBehaviour::getActivitySuppression() const
{
  return activity_suppression_;
}

ImpatiencePtr MotivationalBehaviour::getImpatience() const
{
  return impatience_;
}

ImpatienceResetPtr MotivationalBehaviour::getImpatienceReset() const
{
  return impatience_reset_;
}

InterRobotCommunicationPtr
MotivationalBehaviour::getInterRobotCommunication() const
{
  return monitor_;
}

SensoryFeedbackPtr MotivationalBehaviour::getSensoryFeedback() const
{
  return sensory_feedback_;
}

void MotivationalBehaviour::setThreshold(double threshold,
                                         const ros::Time& timestamp)
{
  if (threshold <= 0.0)
  {
    throw utilities::Exception(
        "The motivational behaviour's activation threshold must be positive.");
  }
  ROS_DEBUG_STREAM("Updating " << *threshold_ << " to " << threshold << ".");
  threshold_->update(threshold, timestamp);
}

void MotivationalBehaviour::setImpatience(double fast_rate,
                                          const ros::Time& timestamp)
{
  impatience_->setFastRate(fast_rate, timestamp);
}

void MotivationalBehaviour::setAcquiescence(
    const ros::Duration& yielding_delay, const ros::Duration& giving_up_delay,
    const ros::Time& timestamp)
{
  acquiescence_->setYieldingDelay(yielding_delay, timestamp);
  acquiescence_->setGivingUpDelay(giving_up_delay, timestamp);
}

bool MotivationalBehaviour::
operator==(const alliance_msgs::Motivation& msg) const
{
  return msg.header.frame_id == robot_->getId() &&
         msg.task_id == behaviour_set_->getTask()->getId();
}
}
