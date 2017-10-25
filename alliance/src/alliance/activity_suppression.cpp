#include "alliance/activity_suppression.h"
#include "alliance/behaviour_set.h"
#include "alliance/robot.h"
#include <typeinfo>
#include <utilities/toggle_event.h>

namespace alliance
{
ActivitySuppression::ActivitySuppression(const RobotPtr& robot,
                                         const BehaviourSetPtr& behaviour_set)
    : Observer::Observer(behaviour_set->getId() + "/activity_suppression"),
      robot_(robot), behaviour_set_(behaviour_set),
      suppressed_(new SampleHolder(behaviour_set_->getId() +
                                       "/activity_suppression/suppressed",
                                   behaviour_set_->getBufferHorizon()))
{
}

ActivitySuppression::~ActivitySuppression() {}

void ActivitySuppression::update(const utilities::EventConstPtr& event)
{
  utilities::ToggleEventConstPtr toggle_event(
      boost::dynamic_pointer_cast<utilities::ToggleEvent const>(event));
  if (toggle_event)
  {
    ROS_DEBUG_STREAM("Updating "
                     << *suppressed_ << " to "
                     << (toggle_event->getValue() ? "true." : "false."));
    suppressed_->update(toggle_event);
  }
}

bool ActivitySuppression::isSuppressed(const ros::Time& timestamp) const
{
  return suppressed_->getValue(timestamp);
}
}
