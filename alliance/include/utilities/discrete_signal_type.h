/**
 *  This header file defines the DiscreteSignalType class, which is based on the
 *NonUnarySignalType abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_DISCRETE_SIGNAL_TYPE_H_
#define _UTILITIES_DISCRETE_SIGNAL_TYPE_H_

#include "utilities/signal_type.h"

namespace utilities
{
class DiscreteSignalType : public SignalType<long>
{
public:
  DiscreteSignalType(long value = 0l);
  DiscreteSignalType(float value);
  DiscreteSignalType(double value);
  DiscreteSignalType(const DiscreteSignalType& signal_type);
  virtual ~DiscreteSignalType();
  virtual bool isDiscrete() const;
  virtual DiscreteSignalType& operator++();
  virtual DiscreteSignalType& operator++(int);
  virtual DiscreteSignalType operator--();
  virtual DiscreteSignalType operator--(int);
};

typedef boost::shared_ptr<DiscreteSignalType> DiscreteSignalTypePtr;
typedef boost::shared_ptr<DiscreteSignalType const> DiscreteSignalTypeConstPtr;
}

#endif // _UTILITIES_DISCRETE_SIGNAL_TYPE_H_
