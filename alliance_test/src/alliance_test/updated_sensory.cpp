#include "alliance_test/updated_sensory.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(alliance_test::UpdatedSensory, alliance::SensoryEvaluator)

namespace alliance_test
{
UpdatedSensory::UpdatedSensory() {}

UpdatedSensory::~UpdatedSensory() {}

bool UpdatedSensory::isApplicable()
{
  for (iterator it(sensors_.begin()); it != sensors_.end(); it++)
  {
    alliance::SensorPtr sensor(*it);
    if (!sensor->isUpToDate())
    {
      return false;
    }
  }
  return true;
}
}
