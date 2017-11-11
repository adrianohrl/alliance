/**
 *  This header file defines and implements the StepFunction class, which is
 *based on the Function abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_STEP_FUNCTION_H_
#define _UTILITIES_STEP_FUNCTION_H_

#include "utilities/continuous_signal_type.h"
#include "utilities/discrete_signal_type.h"
#include "utilities/functions/function.h"
#include "utilities/unary_signal_type.h"

namespace utilities
{
namespace functions
{
template <typename T> class StepFunction : public Function<T>
{
public:
  typedef boost::shared_ptr<StepFunction<T> > Ptr;
  typedef boost::shared_ptr<StepFunction<T> const> ConstPtr;
  StepFunction(double qf, bool ascending = false, bool negated = false);
  StepFunction(double d0, double qf,
               bool ascending = false, bool negated = false);
  StepFunction(double d0, double q0, double qf, bool ascending = false,
               bool negated = false);
  StepFunction(const ros::Duration& d0, double qf, bool ascending = false,
               bool negated = false);
  StepFunction(const ros::Duration& d0, double q0, double qf,
               bool ascending = false, bool negated = false);
  StepFunction(const StepFunction<T>& function);
  virtual ~StepFunction();
  virtual StepFunction<T>* clone() const;

private:
  virtual double calculate(double d) const;
};

typedef StepFunction<utilities::ContinuousSignalType> ContinuousStepFunction;
typedef boost::shared_ptr<ContinuousStepFunction> ContinuousStepFunctionPtr;
typedef boost::shared_ptr<ContinuousStepFunction const>
    ContinuousStepFunctionConstPtr;
typedef StepFunction<utilities::DiscreteSignalType> DiscreteStepFunction;
typedef boost::shared_ptr<DiscreteStepFunction> DiscreteStepFunctionPtr;
typedef boost::shared_ptr<DiscreteStepFunction const>
    DiscreteStepFunctionConstPtr;
typedef StepFunction<utilities::UnarySignalType> UnaryStepFunction;
typedef boost::shared_ptr<UnaryStepFunction> UnaryStepFunctionPtr;
typedef boost::shared_ptr<UnaryStepFunction const> UnaryStepFunctionConstPtr;

template <typename T>
StepFunction<T>::StepFunction(double qf, bool ascending, bool negated)
    : Function<T>::Function("Step", 0.0, INFINITY, 0.0, qf, ascending, negated)
{
}

template <typename T>
StepFunction<T>::StepFunction(double d0, double qf, bool ascending,
                              bool negated)
    : Function<T>::Function("Step", d0, INFINITY, 0.0, qf, ascending, negated)
{
}

template <typename T>
StepFunction<T>::StepFunction(const ros::Duration& d0, double qf,
                              bool ascending, bool negated)
    : Function<T>::Function("Step", d0.toSec(), INFINITY, 0.0, qf, ascending,
                            negated)
{
}

template <typename T>
StepFunction<T>::StepFunction(double d0, double q0, double qf,
                              bool ascending, bool negated)
    : Function<T>::Function("Step", d0, INFINITY, q0, qf, ascending, negated)
{
}

template <typename T>
StepFunction<T>::StepFunction(const ros::Duration& d0, double q0, double qf,
                              bool ascending, bool negated)
    : Function<T>::Function("Step", d0.toSec(), INFINITY, q0, qf, ascending,
                            negated)
{
}

template <typename T>
StepFunction<T>::StepFunction(const StepFunction<T>& function)
    : Function<T>::Function(function)
{
}

template <typename T> StepFunction<T>::~StepFunction() {}

template <typename T> StepFunction<T>* StepFunction<T>::clone() const
{
  return new StepFunction<T>(*this);
}

template <typename T> double StepFunction<T>::calculate(double d) const
{
  return Function<T>::qf_;
}
}
}

#endif // _UTILITIES_STEP_FUNCTION_H_
