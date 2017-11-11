/**
 *  This header file defines the SignalTypeEnum enumerateds and the SignalTypes
 *helper class, which is based on the EnumConverter abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_SIGNAL_TYPES_H_
#define _UTILITIES_SIGNAL_TYPES_H_

#include "utilities/enum_converter.h"

namespace utilities
{
namespace signal_types
{
enum SignalTypeEnum
{
  CONTINUOUS,
  DISCRETE,
  UNARY
};
}

typedef signal_types::SignalTypeEnum SignalTypeEnum;

class SignalTypes : public EnumConverter<SignalTypeEnum>
{
public:
  SignalTypes(SignalTypeEnum enumarated);
  virtual ~SignalTypes();
  virtual SignalTypeEnum getEnumerated(int code) const;
  virtual SignalTypeEnum getEnumerated(std::string name) const;
  virtual int getCode(std::string name) const;
  virtual int getCode(SignalTypeEnum enumerated) const;
  virtual std::string str(SignalTypeEnum enumerated) const;

  static SignalTypeEnum toEnumerated(int code);
  static SignalTypeEnum toEnumerated(std::string name);
  static int toCode(SignalTypeEnum enumerated);
  static std::string toString(SignalTypeEnum enumerated);
  static const char* toCString(SignalTypeEnum enumerated);
  static SignalTypeEnum getDefault();
  static std::vector<SignalTypeEnum> getAll();
  static bool isValid(int code);
  static bool isValid(std::string name);
};
}

#endif // _UTILITIES_SIGNAL_TYPES_H_
