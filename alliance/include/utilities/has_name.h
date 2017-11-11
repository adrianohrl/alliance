/**
 * This header file defines the HasName class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_HAS_NAME_H_
#define _UTILITIES_HAS_NAME_H_

#include "utilities/has_id.h"
#include "utilities/exception.h"

namespace utilities
{
class HasName : public HasId<std::string>
{
public:
  HasName(const std::string& name, const std::string& id = "");
  HasName(const HasName& has_name);
  virtual ~HasName();
  const std::string getName() const;

private:
  const std::string name_;
};
}

#endif // _UTILITIES_HAS_NAME_H_
