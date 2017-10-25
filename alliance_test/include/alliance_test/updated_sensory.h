#ifndef _ALLIANCE_TEST_UPDATED_SENSORY_H_
#define _ALLIANCE_TEST_UPDATED_SENSORY_H_

#include <alliance/sensory_evaluator.h>

namespace alliance_test
{
class UpdatedSensory : public alliance::SensoryEvaluator
{
public:
  UpdatedSensory();
  virtual ~UpdatedSensory();
  virtual bool isApplicable();
};
}

#endif // _ALLIANCE_TEST_UPDATED_SENSORY_H_
