#include "alliance/impatience_reset.h"
#include "alliance/robot.h"

namespace alliance
{
ImpatienceReset::ImpatienceReset() : resetted_(false) {}

ImpatienceReset::~ImpatienceReset() {}

void ImpatienceReset::init(const InterRobotCommunicationPtr& monitor)
{
  if (!monitor_)
  {
    monitor_ = monitor;
  }
}

bool ImpatienceReset::isResetted(const ros::Time& timestamp)
{
  if (resetted_)
  {
    return false;
  }
  ros::Time last_update_timestamp(monitor_->getLastUpdateTimestamp());
  resetted_ =
      monitor_->received(last_update_timestamp, timestamp) &&
      !monitor_->received(ros::Time(), last_update_timestamp);
  return resetted_;
}
}
