/**
 *  This header file defines and implements the ExponentialFunction class, which
 *is based on the Function abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_EXPONENTIAL_FUNCTION_H_
#define _UTILITIES_EXPONENTIAL_FUNCTION_H_

#include <cmath>
#include "utilities/continuous_signal_type.h"
#include "utilities/discrete_signal_type.h"
#include "utilities/functions/function.h"

namespace utilities
{
namespace functions
{
template <typename T> class ExponentialFunction : public Function<T>
{
public:
  typedef boost::shared_ptr<ExponentialFunction<T> > Ptr;
  typedef boost::shared_ptr<ExponentialFunction<T> const> ConstPtr;
  ExponentialFunction(double d0, double df, double q0, double qf,
                      double k = 5.0, double base = M_E, bool ascending = true,
                      bool negated = false);
  ExponentialFunction(const ros::Duration& d0, const ros::Duration& df,
                      double q0, double qf, double k = 5.0,
                      double base = M_E, bool ascending = false,
                      bool negated = false);
  ExponentialFunction(const ExponentialFunction<T>& function);
  virtual ~ExponentialFunction();
  virtual ExponentialFunction<T>* clone() const;

private:
  double base_;
  double k_;
  virtual double calculate(double d) const;
};

typedef ExponentialFunction<utilities::ContinuousSignalType>
    ContinuousExponentialFunction;
typedef boost::shared_ptr<ContinuousExponentialFunction>
    ContinuousExponentialFunctionPtr;
typedef boost::shared_ptr<ContinuousExponentialFunction const>
    ContinuousExponentialFunctionConstPtr;
typedef ExponentialFunction<utilities::DiscreteSignalType>
    DiscreteExponentialFunction;
typedef boost::shared_ptr<DiscreteExponentialFunction>
    DiscreteExponentialFunctionPtr;
typedef boost::shared_ptr<DiscreteExponentialFunction const>
    DiscreteExponentialFunctionConstPtr;

template <typename T>
ExponentialFunction<T>::ExponentialFunction(double d0, double df, double q0,
                                            double qf, double k, double base,
                                            bool ascending, bool negated)
    : Function<T>::Function("Exponential", d0, df, q0, qf, ascending, negated),
      base_(base), k_(fabs(k))
{
}

template <typename T>
ExponentialFunction<T>::ExponentialFunction(const ros::Duration& d0,
                                            const ros::Duration& df,
                                            double q0, double qf, double k,
                                            double base, bool ascending,
                                            bool negated)
    : Function<T>::Function("Exponential", d0, df, q0, qf, ascending, negated),
      base_(base), k_(fabs(k))
{
}

template <typename T>
ExponentialFunction<T>::ExponentialFunction(
    const ExponentialFunction<T>& function)
    : Function<T>::Function(function)
{
}

template <typename T> ExponentialFunction<T>::~ExponentialFunction() {}

template <typename T>
ExponentialFunction<T>* ExponentialFunction<T>::clone() const
{
  return new ExponentialFunction<T>(*this);
}

template <typename T> double ExponentialFunction<T>::calculate(double d) const
{
  double rate(-k_ / (Function<T>::df_ - Function<T>::d0_));
  return Function<T>::qf_ -
         (Function<T>::qf_ - Function<T>::q0_) *
             pow(base_, rate * (d - Function<T>::d0_));
}
}
}

#endif // _UTILITIES_EXPONENTIAL_FUNCTION_H_
