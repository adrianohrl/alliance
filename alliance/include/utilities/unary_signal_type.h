/**
 *  This header file defines the UnarySignalType class, which is based on the
 *SignalType abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_UNARY_SIGNAL_TYPE_H_
#define _UTILITIES_UNARY_SIGNAL_TYPE_H_

#include "utilities/signal_type.h"
#include "utilities/signal_types.h"

namespace utilities
{
class UnarySignalType : public SignalType<bool>
{
public:
  UnarySignalType(bool value = false);
  UnarySignalType(const UnarySignalType& signal_type);
  virtual ~UnarySignalType();
  virtual bool isUnary() const;
  virtual std::string str() const;
  virtual bool operator!() const;
  virtual bool operator&&(const UnarySignalType& signal_type) const;
  virtual bool operator||(const UnarySignalType& signal_type) const;
};

typedef boost::shared_ptr<UnarySignalType> UnarySignalTypePtr;
typedef boost::shared_ptr<UnarySignalType const> UnarySignalTypeConstPtr;
}

#endif // _UTILITIES_UNARY_SIGNAL_TYPE_H_
