/**
 *  This header file defines the ROSMessageConverter interface. It is highly
 *  recommended whenever an oriented-object programming ROS Message class is
 *  created to enhance this one. Implement the following constructor, as well:
 *
 *  class MyClass : public utilities::ROSMessageConverter<T>
 *  {
 *      public:
 *        MyClass(const T& msg);
 *        ...
 *  };
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_ROS_MESSAGE_CONVERTER_H_
#define _UTILITIES_ROS_MESSAGE_CONVERTER_H_

namespace utilities
{
template <typename T> class ROSMessageConverter
{
public:
  virtual T toMsg() const;
  virtual bool operator==(const T& msg) const = 0;
  virtual bool operator!=(const T& msg) const;

protected:
  T msg_;
};

template <typename T>
T ROSMessageConverter<T>::toMsg() const
{
  return msg_;
}

template <typename T>
bool ROSMessageConverter<T>::operator!=(const T& msg) const
{
  return !(*this == msg);
}
}

#endif // _UTILITIES_ROS_MESSAGE_CONVERTER_H_
