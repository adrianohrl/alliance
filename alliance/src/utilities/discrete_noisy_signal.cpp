#include "utilities/discrete_noisy_signal.h"

namespace utilities
{
DiscreteNoisySignal::DiscreteNoisySignal(
    const Interval<DiscreteSignalType>& interval)
    : Noisy<DiscreteSignalType>::Noisy(interval)
{
}

DiscreteNoisySignal::DiscreteNoisySignal(const DiscreteSignalType& min,
                                         const DiscreteSignalType& max)
    : Noisy<DiscreteSignalType>::Noisy(min, max, 3.0)
{
}

DiscreteNoisySignal::DiscreteNoisySignal(const double& mean,
                                         const double& standard_deviation)
    : Noisy<DiscreteSignalType>::Noisy(mean, standard_deviation)
{
}

DiscreteNoisySignal::DiscreteNoisySignal(const DiscreteSignalType& mean,
                                         const double& standard_deviation)
    : Noisy<DiscreteSignalType>::Noisy(mean, standard_deviation)
{
}

DiscreteNoisySignal::DiscreteNoisySignal(const DiscreteNoisySignal& noisy)
    : Noisy<DiscreteSignalType>::Noisy(noisy)
{
}

DiscreteNoisySignal::~DiscreteNoisySignal() {}

double DiscreteNoisySignal::probability(const double& value) const
{
  return ProbabilityDensityFunction::probability(value);
}

double DiscreteNoisySignal::probability(const DiscreteSignalType& value) const
{
  return ProbabilityDensityFunction::probability(value);
}

DiscreteSignalType DiscreteNoisySignal::random()
{
  return DiscreteSignalType(ProbabilityDensityFunction::random());
}

Interval<DiscreteSignalType> DiscreteNoisySignal::getFakeInterval() const
{
  return Interval<DiscreteSignalType>(mean_ - 3 * standard_deviation_,
                                      mean_ + 3 * standard_deviation_);
}
}
