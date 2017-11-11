#ifndef _UTILITIES_CONTINUOUS_NOISY_SIGNAL_H_
#define _UTILITIES_CONTINUOUS_NOISY_SIGNAL_H_

#include "utilities/continuous_signal_type.h"
#include "utilities/noisy.h"

namespace utilities
{
class ContinuousNoisySignal : public Noisy<ContinuousSignalType>
{
public:
  ContinuousNoisySignal(const Interval<ContinuousSignalType>& interval);
  ContinuousNoisySignal(const ContinuousSignalType& min,
                        const ContinuousSignalType& max);
  ContinuousNoisySignal(const double& mean, const double& standard_deviation);
  ContinuousNoisySignal(const ContinuousSignalType& mean,
                        const double& standard_deviation);
  ContinuousNoisySignal(const ContinuousNoisySignal& noisy);
  virtual ~ContinuousNoisySignal();
  virtual double probability(const double& value) const;
  virtual double probability(const ContinuousSignalType& value) const;
  virtual ContinuousSignalType random();
  virtual Interval<ContinuousSignalType> getFakeInterval() const;
};

typedef boost::shared_ptr<ContinuousNoisySignal> ContinuousNoisySignalPtr;
typedef boost::shared_ptr<ContinuousNoisySignal const>
    ContinuousNoisySignalConstPtr;
}

#endif // _UTILITIES_CONTINUOUS_NOISY_SIGNAL_H_
