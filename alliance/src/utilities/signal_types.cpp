/**
 *  This source file implements the SignalTypes helper class, which is based on
 *the EnumConverter abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/signal_types.h"
#include "utilities/exception.h"

namespace utilities
{
SignalTypes::SignalTypes(SignalTypeEnum enumarated) : EnumConverter(enumarated)
{
}

SignalTypes::~SignalTypes() {}

SignalTypeEnum SignalTypes::getEnumerated(int code) const
{
  return SignalTypes::toEnumerated(code);
}

SignalTypeEnum SignalTypes::getEnumerated(std::string name) const
{
  return SignalTypes::toEnumerated(name);
}

int SignalTypes::getCode(std::string name) const
{
  return SignalTypes::toCode(SignalTypes::toEnumerated(name));
}

int SignalTypes::getCode(SignalTypeEnum enumerated) const
{
  return SignalTypes::toCode(enumerated);
}

std::string SignalTypes::str(SignalTypeEnum enumerated) const
{
  return SignalTypes::toString(enumerated);
}

SignalTypeEnum SignalTypes::toEnumerated(int code)
{
  SignalTypeEnum enumerated;
  switch (code)
  {
  case 0:
    enumerated = signal_types::CONTINUOUS;
    break;
  case 1:
    enumerated = signal_types::DISCRETE;
    break;
  case 2:
    enumerated = signal_types::UNARY;
    break;
  default:
    // enumerated = SignalTypes::getDefault();
    throw utilities::Exception("Invalid signal type code.");
  }
  return enumerated;
}

SignalTypeEnum SignalTypes::toEnumerated(std::string name)
{
  SignalTypeEnum enumerated;
  if (name == "CONTINUOUS" || name == "Continuous" || name == "continuous")
  {
    enumerated = signal_types::CONTINUOUS;
  }
  else if (name == "DISCRETE" || name == "Discrete" || name == "discrete")
  {
    enumerated = signal_types::DISCRETE;
  }
  else if (name == "UNARY" || name == "Unary" || name == "unary")
  {
    enumerated = signal_types::UNARY;
  }
  else
  {
    // enumerated = SignalTypes::getDefault();
    throw utilities::Exception("Invalid signal type name.");
  }
  return enumerated;
}

int SignalTypes::toCode(SignalTypeEnum enumerated)
{
  int code;
  switch (enumerated)
  {
  case signal_types::CONTINUOUS:
    code = 0;
    break;
  case signal_types::DISCRETE:
    code = 1;
    break;
  case signal_types::UNARY:
    code = 2;
    break;
  default:
    // code = SignalTypes::toCode(SignalTypes::getDefault());
    throw utilities::Exception("Invalid signal type enumerated.");
  }
  return code;
}

std::string SignalTypes::toString(SignalTypeEnum enumerated)
{
  std::string name;
  switch (enumerated)
  {
  case signal_types::CONTINUOUS:
    name = "CONTINUOUS";
    break;
  case signal_types::DISCRETE:
    name = "DISCRETE";
    break;
  case signal_types::UNARY:
    name = "UNARY";
    break;
  default:
    // name = SignalTypes::toString(SignalTypes::getDefault());
    throw utilities::Exception("Invalid signal type enumerated.");
  }
  return name;
}

const char* SignalTypes::toCString(SignalTypeEnum enumerated)
{
  return SignalTypes::toString(enumerated).c_str();
}

SignalTypeEnum SignalTypes::getDefault() { return signal_types::CONTINUOUS; }

std::vector<SignalTypeEnum> SignalTypes::getAll()
{
  std::vector<SignalTypeEnum> types;
  types.push_back(signal_types::CONTINUOUS);
  types.push_back(signal_types::DISCRETE);
  types.push_back(signal_types::UNARY);
  return types;
}

bool SignalTypes::isValid(int code) { return code >= 0 && code <= 2; }

bool SignalTypes::isValid(std::string name)
{
  try
  {
    toEnumerated(name);
  }
  catch (utilities::Exception e)
  {
    return false;
  }
  return true;
}
}
