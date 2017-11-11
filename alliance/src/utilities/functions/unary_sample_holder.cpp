#include "utilities/functions/unary_sample_holder.h"
#include "utilities/functions/step_function.h"

namespace utilities
{
namespace functions
{
UnarySampleHolder::UnarySampleHolder(const std::string& id,
                                     const ros::Duration& buffer_horizon,
                                     const ros::Time& start_timestamp)
    : SampleHolder<UnarySignalType>::SampleHolder(
          id, UnaryStepFunctionPtr(new UnaryStepFunction(1.0, true)),
          buffer_horizon, start_timestamp)
{
}

UnarySampleHolder::UnarySampleHolder(const std::string& id,
                                     const ros::Duration& timeout_duration,
                                     const ros::Duration& buffer_horizon,
                                     const ros::Time& start_timestamp)
    : SampleHolder<UnarySignalType>::SampleHolder(
          id, UnaryStepFunctionPtr(new UnaryStepFunction(1.0, true)),
          timeout_duration, buffer_horizon, start_timestamp)
{
}

UnarySampleHolder::UnarySampleHolder(const UnarySampleHolder& function)
    : SampleHolder<UnarySignalType>::SampleHolder(function)
{
}

UnarySampleHolder::~UnarySampleHolder() {}

void UnarySampleHolder::update(const ToggleEventConstPtr& event)
{
  SampleHolder<UnarySignalType>::update(event->getValue(),
                                        event->getTimestamp());
}

bool UnarySampleHolder::updated(const ros::Time& t1, const ros::Time& t2) const
{
  ros::Time t0(BufferedFunction<UnarySignalType>::getStartTimestamp());
  if (t1 > t2)
  {
    throw utilities::Exception("t2 must be greater than t1.");
  }
  ros::Duration d1;
  if (t1 > t0)
  {
    d1 = t1 - t0;
  }
  ros::Duration d2(t2 - t0);
  ros::Duration delta_d(
      0.1 * BufferedFunction<UnarySignalType>::getTimeoutDuration().toSec());
  for (ros::Duration d(d1); d <= d2; d += delta_d)
  {
    if (BufferedFunction<UnarySignalType>::getValue(d.toSec()))
    {
      return true;
    }
  }
  return false;
}
}
}
