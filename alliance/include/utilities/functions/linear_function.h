/**
 *  This header file defines and implements the LinearFunction class, which is
 *based on the Function abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_LINEAR_FUNCTION_H_
#define _UTILITIES_LINEAR_FUNCTION_H_

#include "utilities/continuous_signal_type.h"
#include "utilities/discrete_signal_type.h"
#include "utilities/functions/function.h"

namespace utilities
{
namespace functions
{
template <typename T> class LinearFunction : public Function<T>
{
public:
  typedef boost::shared_ptr<LinearFunction<T> > Ptr;
  typedef boost::shared_ptr<LinearFunction<T> const> ConstPtr;
  LinearFunction(double d0, double df, double q0, double qf,
                 bool ascending = false, bool negated = false);
  LinearFunction(const ros::Duration& d0, const ros::Duration& df, double q0,
                 double qf, bool ascending = false, bool negated = false);
  LinearFunction(const LinearFunction<T>& function);
  virtual ~LinearFunction();
  virtual LinearFunction<T>* clone() const;

protected:
private:
  virtual double calculate(double d) const;
};

typedef LinearFunction<utilities::ContinuousSignalType>
    ContinuousLinearFunction;
typedef boost::shared_ptr<ContinuousLinearFunction> ContinuousLinearFunctionPtr;
typedef boost::shared_ptr<ContinuousLinearFunction const>
    ContinuousLinearFunctionConstPtr;
typedef LinearFunction<utilities::DiscreteSignalType> DiscreteLinearFunction;
typedef boost::shared_ptr<DiscreteLinearFunction> DiscreteLinearFunctionPtr;
typedef boost::shared_ptr<DiscreteLinearFunction const>
    DiscreteLinearFunctionConstPtr;

template <typename T>
LinearFunction<T>::LinearFunction(double d0, double df, double q0,
                                  double qf, bool ascending, bool negated)
    : Function<T>::Function("Linear", d0, df, q0, qf, ascending, negated)
{
}

template <typename T>
LinearFunction<T>::LinearFunction(const ros::Duration& d0,
                                  const ros::Duration& df, double q0,
                                  double qf, bool ascending, bool negated)
    : Function<T>::Function("Linear", d0, df, q0, qf, ascending, negated)
{
}

template <typename T>
LinearFunction<T>::LinearFunction(const LinearFunction<T>& function)
    : Function<T>::Function(function)
{
}

template <typename T> LinearFunction<T>* LinearFunction<T>::clone() const
{
  return new LinearFunction<T>(*this);
}

template <typename T> LinearFunction<T>::~LinearFunction() {}

template <typename T> double LinearFunction<T>::calculate(double d) const
{
  double rate((Function<T>::qf_ - Function<T>::q0_) /
              (Function<T>::df_ - Function<T>::d0_));
  return rate * (d - Function<T>::d0_) + Function<T>::q0_;
}
}
}

#endif // _UTILITIES_LINEAR_FUNCTION_H_
