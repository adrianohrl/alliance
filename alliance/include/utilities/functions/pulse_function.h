/**
 *  This header file defines and implements the PulseFunction class, which is
 *based on the Function abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_PULSE_FUNCTION_H_
#define _UTILITIES_PULSE_FUNCTION_H_

#include "utilities/functions/function.h"
#include "utilities/functions/step_function.h"

namespace utilities
{
namespace functions
{
template <typename T> class PulseFunction : public Function<T>
{
public:
  typedef boost::shared_ptr<PulseFunction<T> > Ptr;
  typedef boost::shared_ptr<PulseFunction<T> const> ConstPtr;
  PulseFunction(double d0, double df, double qf, bool ascending = false,
                bool negated = false);
  PulseFunction(double d0, double df, double q0, double qf,
                bool ascending = false, bool negated = false);
  PulseFunction(const ros::Duration& d0, const ros::Duration& df, double qf,
                bool ascending = false, bool negated = false);
  PulseFunction(const ros::Duration& d0, const ros::Duration& df, double q0,
                double qf, bool ascending = false, bool negated = false);
  PulseFunction(const StepFunction<T>& step_function, double df);
  PulseFunction(const StepFunction<T>& step_function, const ros::Duration& df);
  PulseFunction(const PulseFunction<T>& function);
  virtual ~PulseFunction();
  virtual PulseFunction<T>* clone() const;

protected:
private:
  virtual double calculate(double d) const;
};

typedef PulseFunction<utilities::ContinuousSignalType> ContinuousPulseFunction;
typedef boost::shared_ptr<ContinuousPulseFunction> ContinuousPulseFunctionPtr;
typedef boost::shared_ptr<ContinuousPulseFunction const>
    ContinuousPulseFunctionConstPtr;
typedef PulseFunction<utilities::DiscreteSignalType> DiscretePulseFunction;
typedef boost::shared_ptr<DiscretePulseFunction> DiscretePulseFunctionPtr;
typedef boost::shared_ptr<DiscretePulseFunction const>
    DiscretePulseFunctionConstPtr;
typedef PulseFunction<utilities::UnarySignalType> UnaryPulseFunction;
typedef boost::shared_ptr<UnaryPulseFunction> UnaryPulseFunctionPtr;
typedef boost::shared_ptr<UnaryPulseFunction const> UnaryPulseFunctionConstPtr;

template <typename T>
PulseFunction<T>::PulseFunction(double d0, double df, double qf,
                                bool ascending, bool negated)
    : Function<T>::Function("Pulse", d0, df, 0.0, qf, ascending, false, negated)
{
  if (d0 == 0.0 || df >= INFINITY)
  {
    throw utilities::Exception(
        "Use the step function instead of the pulse one.");
  }
}

template <typename T>
PulseFunction<T>::PulseFunction(const ros::Duration& d0,
                                const ros::Duration& df, double qf,
                                bool ascending, bool negated)
    : Function<T>::Function("Pulse", d0, df, 0.0, qf, ascending, false, negated)
{
  if (d0.toSec() == 0.0 || df.toSec() >= INFINITY)
  {
    throw utilities::Exception(
        "Use the step function instead of the pulse one.");
  }
}

template <typename T>
PulseFunction<T>::PulseFunction(const StepFunction<T>& step_function, double df)
    : Function<T>::Function("Pulse", step_function, false)
{
  Function<T>::df_ = df;
}

template <typename T>
PulseFunction<T>::PulseFunction(const StepFunction<T>& step_function,
                                const ros::Duration& df)
    : Function<T>::Function("Pulse", step_function, false)
{
  Function<T>::df_ = df.toSec();
}

template <typename T>
PulseFunction<T>::PulseFunction(double d0, double df, double q0, double qf,
                                bool ascending, bool negated)
    : Function<T>::Function("Pulse", d0, df, q0, qf, ascending, negated, false)
{
  if (d0 == 0.0 || df >= INFINITY)
  {
    throw utilities::Exception(
        "Use the step function instead of the pulse one.");
  }
}

template <typename T>
PulseFunction<T>::PulseFunction(const ros::Duration& d0,
                                const ros::Duration& df, double q0,
                                double qf, bool ascending, bool negated)
    : Function<T>::Function("Pulse", d0, df, q0, qf, ascending, negated, false)
{
  if (d0.toSec() == 0.0 || df.toSec() >= INFINITY)
  {
    throw utilities::Exception(
        "Use the step function instead of the pulse one.");
  }
}

template <typename T>
PulseFunction<T>::PulseFunction(const PulseFunction<T>& function)
    : Function<T>::Function(function)
{
}

template <typename T> PulseFunction<T>::~PulseFunction() {}

template <typename T> PulseFunction<T>* PulseFunction<T>::clone() const
{
  return new PulseFunction<T>(*this);
}

template <typename T> double PulseFunction<T>::calculate(double d) const
{
  return d <= Function<T>::d0_ || d > Function<T>::df_ ? Function<T>::q0_
                                                       : Function<T>::qf_;
}
}
}

#endif // _UTILITIES_PULSE_FUNCTION_H_
