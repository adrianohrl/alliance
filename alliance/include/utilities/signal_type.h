/**
 *  This header file defines the SignalType abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_SIGNAL_TYPE_H_
#define _UTILITIES_SIGNAL_TYPE_H_

#include <boost/shared_ptr.hpp>
#include <sstream>

namespace utilities
{
template <typename T> class SignalType
{
public:
  SignalType(const T& value);
  SignalType(const SignalType<T>& signal_type);
  virtual ~SignalType();
  virtual bool isContinuous() const;
  virtual bool isDiscrete() const;
  virtual bool isUnary() const;
  T getValue() const;
  void setValue(const T& value);
  virtual std::string str() const;
  const char* c_str() const;
  /*virtual bool operator<(const T& value) const;
  virtual bool operator<(const SignalType<T>& signal_type) const;
  virtual bool operator<=(const T& value) const;
  virtual bool operator<=(const SignalType<T>& signal_type) const;
  virtual bool operator==(const T& value) const;
  virtual bool operator==(const SignalType<T>& signal_type) const;
  virtual bool operator!=(const T& value) const;
  virtual bool operator!=(const SignalType<T>& signal_type) const;
  virtual bool operator>=(const T& value) const;
  virtual bool operator>=(const SignalType<T>& signal_type) const;
  virtual bool operator>(const T& value) const;
  virtual bool operator>(const SignalType<T>& signal_type) const;
  virtual SignalType<T>& operator+(const T& value);
  virtual SignalType<T>& operator+(const SignalType<T>& signal_type);
  virtual SignalType<T>& operator-(const T& value);
  virtual SignalType<T>& operator-(const SignalType<T>& signal_type);
  virtual SignalType<T>& operator*(const T& value);
  virtual SignalType<T>& operator*(const SignalType<T>& signal_type);
  virtual SignalType<T>& operator/(const T& value);
  virtual SignalType<T>& operator/(const SignalType<T>& signal_type);*/
  virtual SignalType<T>& operator=(const T& value);
  virtual SignalType<T>& operator=(const SignalType<T>& signal_type);
  virtual SignalType<T>& operator+=(const T& value);
  virtual SignalType<T>& operator+=(const SignalType<T>& signal_type);
  virtual SignalType<T>& operator-=(const T& value);
  virtual SignalType<T>& operator-=(const SignalType<T>& signal_type);
  virtual SignalType<T>& operator*=(const T& value);
  virtual SignalType<T>& operator*=(const SignalType<T>& signal_type);
  virtual SignalType<T>& operator/=(const T& value);
  virtual SignalType<T>& operator/=(const SignalType<T>& signal_type);
  virtual operator T() const;
  template <typename U>
  friend std::ostream& operator<<(std::ostream& out,
                                  const SignalType<U>& signal_type);

protected:
  T value_;
};

template <typename T> SignalType<T>::SignalType(const T& value) : value_(value)
{
}

template <typename T>
SignalType<T>::SignalType(const SignalType<T>& signal_type)
    : value_(signal_type.value_)
{
}

template <typename T> SignalType<T>::~SignalType() {}

template <typename T> bool SignalType<T>::isContinuous() const { return false; }

template <typename T> bool SignalType<T>::isDiscrete() const { return false; }

template <typename T> bool SignalType<T>::isUnary() const { return false; }

template <typename T> T SignalType<T>::getValue() const { return value_; }

template <typename T> void SignalType<T>::setValue(const T& value)
{
  value_ = value;
}

template <typename T> std::string SignalType<T>::str() const
{
  std::stringstream ss;
  ss << value_;
  return ss.str();
}

template <typename T> const char* SignalType<T>::c_str() const
{
  return str().c_str();
}
/*
template <typename T> bool SignalType<T>::operator<(const T& value) const
{
  return value_ < value;
}

template <typename T>
bool SignalType<T>::operator<(const SignalType<T>& signal_type) const
{
  return value_ < signal_type.value_;
}

template <typename T> bool SignalType<T>::operator<=(const T& value) const
{
  return value_ <= value;
}

template <typename T>
bool SignalType<T>::operator<=(const SignalType<T>& signal_type) const
{
  return value_ <= signal_type.value_;
}

template <typename T> bool SignalType<T>::operator==(const T& value) const
{
  return value_ == value;
}

template <typename T>
bool SignalType<T>::operator==(const SignalType<T>& signal_type) const
{
  return value_ == signal_type.value_;
}

template <typename T> bool SignalType<T>::operator!=(const T& value) const
{
  return value_ != value;
}

template <typename T>
bool SignalType<T>::operator!=(const SignalType<T>& signal_type) const
{
  return value_ != signal_type.value_;
}

template <typename T> bool SignalType<T>::operator>=(const T& value) const
{
  return value_ >= value;
}

template <typename T>
bool SignalType<T>::operator>=(const SignalType<T>& signal_type) const
{
  return value_ >= signal_type.value_;
}

template <typename T> bool SignalType<T>::operator>(const T& value) const
{
  return value_ > value;
}

template <typename T>
bool SignalType<T>::operator>(const SignalType<T>& signal_type) const
{
  return value_ > signal_type.value_;
}

/*template <typename T> SignalType<T>& SignalType<T>::operator-()
{
  value_ = -value_;
  return *this;
}*//*

template <typename T> SignalType<T>& SignalType<T>::operator+(const T& value)
{
  value_ += value;
  return *this;
}

template <typename T>
SignalType<T>& SignalType<T>::operator+(const SignalType<T>& signal_type)
{
  value_ += signal_type.value_;
  return *this;
}

template <typename T> SignalType<T>& SignalType<T>::operator-(const T& value)
{
  value_ -= value;
  return *this;
}

template <typename T>
SignalType<T>& SignalType<T>::operator-(const SignalType<T>& signal_type)
{
  value_ -= signal_type.value_;
  return *this;
}

template <typename T> SignalType<T>& SignalType<T>::operator*(const T& value)
{
  value_ *= value;
  return *this;
}

template <typename T>
SignalType<T>& SignalType<T>::operator*(const SignalType<T>& signal_type)
{
  value_ *= signal_type.value_;
  return *this;
}

template <typename T> SignalType<T>& SignalType<T>::operator/(const T& value)
{
  value_ /= value;
  return *this;
}

template <typename T>
SignalType<T>& SignalType<T>::operator/(const SignalType<T>& signal_type)
{
  value_ /= signal_type.value_;
  return *this;
}
*/
template <typename T> SignalType<T>& SignalType<T>::operator=(const T& value)
{
  value_ = value;
  return *this;
}

template <typename T>
SignalType<T>& SignalType<T>::operator=(const SignalType<T>& signal_type)
{
  value_ = signal_type.value_;
  return *this;
}

template <typename T> SignalType<T>& SignalType<T>::operator+=(const T& value)
{
  value_ += value;
  return *this;
}

template <typename T>
SignalType<T>& SignalType<T>::operator+=(const SignalType<T>& signal_type)
{
  value_ += signal_type.value_;
  return *this;
}

template <typename T> SignalType<T>& SignalType<T>::operator-=(const T& value)
{
  value_ -= value;
  return *this;
}

template <typename T>
SignalType<T>& SignalType<T>::operator-=(const SignalType<T>& signal_type)
{
  value_ -= signal_type.value_;
  return *this;
}

template <typename T> SignalType<T>& SignalType<T>::operator*=(const T& value)
{
  value_ *= value;
  return *this;
}

template <typename T>
SignalType<T>& SignalType<T>::operator*=(const SignalType<T>& signal_type)
{
  value_ *= signal_type.value_;
  return *this;
}

template <typename T> SignalType<T>& SignalType<T>::operator/=(const T& value)
{
  value_ /= value;
  return *this;
}

template <typename T>
SignalType<T>& SignalType<T>::operator/=(const SignalType<T>& signal_type)
{
  value_ /= signal_type.value_;
  return *this;
}

template <typename T> SignalType<T>::operator T() const { return value_; }

template <typename T>
std::ostream& operator<<(std::ostream& out, const SignalType<T>& signal_type)
{
  out << signal_type.getValue();
  return out;
}
}

#endif // _UTILITIES_SIGNAL_TYPE_H_
