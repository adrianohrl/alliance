/**
 *  This source file implements and implements the Observer abstract class of
 *the Observer Design Pattern.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/observer.h"
#include "utilities/exception.h"

namespace utilities
{
Observer::Observer(const std::string& id) : HasId<std::string>::HasId(id)
{
  if (id.empty())
  {
    throw utilities::Exception("Observer id must not be empty.");
  }
}

Observer::Observer(const Observer& observer)
    : HasId<std::string>::HasId(observer)
{
}

Observer::~Observer() {}
}
