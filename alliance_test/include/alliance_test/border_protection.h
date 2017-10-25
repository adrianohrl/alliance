#ifndef _ALLIANCE_TEST_BORDER_PROTECTION_H_
#define _ALLIANCE_TEST_BORDER_PROTECTION_H_

#include "alliance_test/layer.h"

namespace alliance_test
{
class BorderProtection : public Layer
{
public:
  BorderProtection();
  virtual ~BorderProtection();
  virtual void process();
  virtual void initialize(const std::string& ns, const std::string& name);

private:
  bool align_left_;
  bool align_right_;
  double x_min_;
  double y_min_;
  double x_max_;
  double y_max_;
  static const double DANGEROUS_DISTANCE = 0.25;
  static const double GAIN = 0.25;
};
}

#endif // _ALLIANCE_TEST_BORDER_PROTECTION_H_
