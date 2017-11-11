#ifndef _UTILITIES_DISCRETE_NOISY_SIGNAL_H_
#define _UTILITIES_DISCRETE_NOISY_SIGNAL_H_

#include "utilities/discrete_signal_type.h"
#include "utilities/noisy.h"

namespace utilities
{
class DiscreteNoisySignal : public Noisy<DiscreteSignalType>
{
public:
  DiscreteNoisySignal(const Interval<DiscreteSignalType>& interval);
  DiscreteNoisySignal(const DiscreteSignalType& min,
                      const DiscreteSignalType& max);
  DiscreteNoisySignal(const double& mean, const double& standard_deviation);
  DiscreteNoisySignal(const DiscreteSignalType& mean,
                      const double& standard_deviation);
  DiscreteNoisySignal(const DiscreteNoisySignal& noisy);
  virtual ~DiscreteNoisySignal();
  virtual double probability(const double& value) const;
  virtual double probability(const DiscreteSignalType& value) const;
  virtual DiscreteSignalType random();
  virtual Interval<DiscreteSignalType> getFakeInterval() const;
};

typedef boost::shared_ptr<DiscreteNoisySignal> DiscreteNoisySignalPtr;
typedef boost::shared_ptr<DiscreteNoisySignal const>
    DiscreteNoisySignalConstPtr;
}

#endif // _UTILITIES_DISCRETE_NOISY_SIGNAL_H_
