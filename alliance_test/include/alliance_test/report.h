#ifndef _ALLIANCE_TEST_REPORT_H_
#define _ALLIANCE_TEST_REPORT_H_

#include "alliance_test/layer.h"

namespace alliance_test
{
class Report : public Layer
{
public:
  Report();
  virtual ~Report();
  virtual void process();
};
}

#endif // _ALLIANCE_TEST_REPORT_H_
