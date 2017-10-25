#ifndef _ALLIANCE_TEST_WANDER_H_
#define _ALLIANCE_TEST_WANDER_H_

#include "alliance_test/layer.h"

namespace alliance_test
{
class Wander : public Layer
{
public:
  Wander();
  virtual ~Wander();
  virtual void process();

private:
  static const double SAFE_DISTANCE = 1.5;
  static const double TOLERANCE = 5e-2;
  static const double GAIN = 0.5;
  bool isSafe() const;
  bool isInDanger() const;
};
}

#endif // _ALLIANCE_TEST_WANDER_H_
