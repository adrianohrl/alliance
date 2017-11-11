#include "utilities/continuous_noisy_signal.h"

namespace utilities
{

ContinuousNoisySignal::ContinuousNoisySignal(
    const Interval<ContinuousSignalType>& interval)
    : Noisy<ContinuousSignalType>::Noisy(interval)
{
}

ContinuousNoisySignal::ContinuousNoisySignal(const ContinuousSignalType& min,
                                             const ContinuousSignalType& max)
    : Noisy<ContinuousSignalType>::Noisy(min, max, 3.0)
{
}

ContinuousNoisySignal::ContinuousNoisySignal(const double& mean,
                                             const double& standard_deviation)
    : Noisy<ContinuousSignalType>::Noisy(mean, standard_deviation)
{
}

ContinuousNoisySignal::ContinuousNoisySignal(const ContinuousSignalType& mean,
                                             const double& standard_deviation)
    : Noisy<ContinuousSignalType>::Noisy(mean, standard_deviation)
{
}

ContinuousNoisySignal::ContinuousNoisySignal(const ContinuousNoisySignal& noisy)
    : Noisy<ContinuousSignalType>::Noisy(noisy)
{
}

ContinuousNoisySignal::~ContinuousNoisySignal() {}

double ContinuousNoisySignal::probability(const double& value) const
{
  return ProbabilityDensityFunction::probability(value);
}

double
ContinuousNoisySignal::probability(const ContinuousSignalType& value) const
{
  return ProbabilityDensityFunction::probability(value);
}

ContinuousSignalType ContinuousNoisySignal::random()
{
  return ContinuousSignalType(ProbabilityDensityFunction::random());
}

Interval<ContinuousSignalType> ContinuousNoisySignal::getFakeInterval() const
{
  return Interval<ContinuousSignalType>(mean_ - 3 * standard_deviation_,
                                        mean_ + 3 * standard_deviation_);
}
}
