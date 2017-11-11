/**
 * This source file implements the HasName class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/has_name.h"

namespace utilities
{
HasName::HasName(const std::string& name, const std::string& id)
    : HasId<std::string>::HasId(id.empty() ? name : id), name_(name)
{
  if (id.empty())
  {
    throw utilities::Exception("Id must not be empty.");
  }
  if (name.empty())
  {
    throw utilities::Exception("Name must not be empty.");
  }
}

HasName::HasName(const HasName& has_name)
    : HasId<std::string>::HasId(has_name), name_(has_name.name_)
{
}

HasName::~HasName() {}

const std::string HasName::getName() const { return name_; }
}
