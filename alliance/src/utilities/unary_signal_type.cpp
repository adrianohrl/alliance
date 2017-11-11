/**
 *  This source file implements the UnarySignalType class, which is based on the
 *SignalType abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/unary_signal_type.h"

namespace utilities
{

UnarySignalType::UnarySignalType(bool value)
    : SignalType<bool>::SignalType(value)
{
}

UnarySignalType::UnarySignalType(const UnarySignalType& signal_type)
    : SignalType<bool>::SignalType(signal_type)
{
}

UnarySignalType::~UnarySignalType() {}

bool UnarySignalType::isUnary() const { return true; }

std::string UnarySignalType::str() const { return value_ ? "true" : "false"; }

bool UnarySignalType::operator!() const { return !value_; }

bool UnarySignalType::operator&&(const UnarySignalType& signal_type) const
{
  return value_ && signal_type.value_;
}

bool UnarySignalType::operator||(const UnarySignalType& signal_type) const
{
  return value_ || signal_type.value_;
}
}
