/**
 * This header file defines the HasId class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_HAS_ID_H_
#define _UTILITIES_HAS_ID_H_

#include <sstream>

namespace utilities
{
template <typename K> class HasId
{
public:
  virtual ~HasId();
  const K getId() const;
  std::string str() const;
  const char* c_str() const;
  bool operator<(const HasId<K>& has_id);
  bool operator<=(const HasId<K>& has_id);
  bool operator==(const HasId<K>& has_id);
  bool operator!=(const HasId<K>& has_id);
  bool operator>=(const HasId<K>& has_id);
  bool operator>(const HasId<K>& has_id);
  template <typename U>
  friend std::ostream& operator<<(std::ostream& out, const HasId<U>& has_id);

protected:
  const K id_;
  HasId(const K& id);
  HasId(const HasId<K>& has_id);
  void setId(const K& id);
};

template <typename K> HasId<K>::HasId(const K& id) : id_(id) {}

template <typename K> HasId<K>::HasId(const HasId<K>& has_id) : id_(has_id.id_)
{
}

template <typename K> void HasId<K>::setId(const K& id) { id_ = id; }

template <typename K> HasId<K>::~HasId() {}

template <typename K> const K HasId<K>::getId() const { return id_; }

template <typename K> std::string HasId<K>::str() const
{
  std::stringstream ss;
  ss << id_;
  return ss.str();
}

template <typename K> const char* HasId<K>::c_str() const
{
  return str().c_str();
}

template <typename K> bool HasId<K>::operator<(const HasId<K>& has_id)
{
  return id_ < has_id.id_;
}

template <typename K> bool HasId<K>::operator<=(const HasId<K>& has_id)
{
  return id_ <= has_id.id_;
}

template <typename K> bool HasId<K>::operator==(const HasId<K>& has_id)
{
  return id_ == has_id.id_;
}

template <typename K> bool HasId<K>::operator!=(const HasId<K>& has_id)
{
  return id_ != has_id.id_;
}

template <typename K> bool HasId<K>::operator>=(const HasId<K>& has_id)
{
  return id_ >= has_id.id_;
}

template <typename K> bool HasId<K>::operator>(const HasId<K>& has_id)
{
  return id_ > has_id.id_;
}

template <typename K>
std::ostream& operator<<(std::ostream& out, const HasId<K>& has_id)
{
  out << has_id.id_;
  return out;
}
}

#endif // _UTILITIES_HAS_ID_H_
