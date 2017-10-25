#include "alliance/sensory_feedback.h"
#include "alliance/robot.h"

namespace alliance
{
SensoryFeedback::SensoryFeedback(const RobotPtr& robot,
                                 const BehaviourSetPtr& behaviour_set)
    : AllianceObserver<alliance_msgs::SensoryFeedback>::AllianceObserver(
          behaviour_set->getId() + "/sensory_feedback"),
      robot_(robot), behaviour_set_(behaviour_set),
      applicable_(new SampleHolder(
          behaviour_set_->getId() + "/sensory_feedback/applicable",
          robot_->getTimeoutDuration(), behaviour_set_->getBufferHorizon()))
{
}

SensoryFeedback::~SensoryFeedback() {}

void SensoryFeedback::update(const nodes::SensoryFeedbackEventConstPtr& event)
{
  if (!event->isRelated(*behaviour_set_->getTask()))
  {
    return;
  }
  alliance_msgs::SensoryFeedback msg = event->getMsg();
  ROS_DEBUG_STREAM("Updating " << *applicable_ << " to "
                               << (msg.applicable ? "true." : "false."));
  applicable_->update(msg.applicable, event->getTimestamp());
}

bool SensoryFeedback::isApplicable(const ros::Time& timestamp)
{
  return applicable_->getValue(timestamp);
}
}
