#ifndef _UTILITIES_BUFFERED_FUNCTION_H_
#define _UTILITIES_BUFFERED_FUNCTION_H_

#include <list>
#include <ros/node_handle.h>
#include "utilities/observer.h"
#include "utilities/functions/pulse_function.h"

namespace utilities
{
namespace functions
{
template <typename T> class BufferedFunction : public Observer
{
private:
  typedef typename Function<T>::Ptr FunctionPtr;

public:
  typedef typename boost::shared_ptr<BufferedFunction<T> > Ptr;
  typedef typename boost::shared_ptr<BufferedFunction<T> const> ConstPtr;
  BufferedFunction(const std::string& id, const FunctionPtr& model,
                   const ros::Duration& buffer_horizon,
                   const ros::Time& start_timestamp = ros::Time::now());
  BufferedFunction(const std::string& id, const FunctionPtr& model,
                   const ros::Duration& timeout_duration,
                   const ros::Duration& buffer_horizon,
                   const ros::Time& start_timestamp = ros::Time::now());
  BufferedFunction(const BufferedFunction<T>& function);
  virtual ~BufferedFunction();
  ros::Time getStartTimestamp() const;
  ros::Duration getTimeoutDuration() const;
  ros::Duration getBufferHorizon() const;
  bool expires() const;
  T getValue(const ros::Time& timestamp = ros::Time::now());
  virtual void update(const EventConstPtr& event);
  void update(const EventConstPtr& event, const FunctionPtr& model);
  virtual void update(const ros::Time& timestamp);
  void update(const ros::Time& timestamp, const FunctionPtr& model);
  ros::Time getLastUpdateTimestamp() const;
  void setTimeoutDuration(const ros::Duration& timeout_duration);
  void setBufferHorizon(const ros::Duration& buffer_horizon);

protected:
  T getValue(double d) const;

private:
  typedef typename std::list<FunctionPtr>::iterator iterator;
  typedef typename std::list<FunctionPtr>::const_iterator const_iterator;
  ros::Time start_timestamp_;
  ros::Time last_update_timestamp_;
  ros::Duration timeout_duration_;
  ros::Duration buffer_horizon_;
  std::list<FunctionPtr> functions_;
  FunctionPtr model_;
  void cleanBuffer(double d);
};

template <typename T>
BufferedFunction<T>::BufferedFunction(const std::string& id,
                                      const FunctionPtr& model,
                                      const ros::Duration& buffer_horizon,
                                      const ros::Time& start_timestamp)
    : Observer::Observer(id), model_(model), start_timestamp_(start_timestamp),
      buffer_horizon_(buffer_horizon)
{
}

template <typename T>
BufferedFunction<T>::BufferedFunction(const std::string& id,
                                      const FunctionPtr& model,
                                      const ros::Duration& timeout_duration,
                                      const ros::Duration& buffer_horizon,
                                      const ros::Time& start_timestamp)
    : Observer::Observer(id), model_(model), start_timestamp_(start_timestamp),
      timeout_duration_(timeout_duration), buffer_horizon_(buffer_horizon)
{
}

template <typename T>
BufferedFunction<T>::BufferedFunction(const BufferedFunction<T>& function)
    : Observer::Observer(function), model_(function.model_),
      start_timestamp_(ros::Time::now()),
      timeout_duration_(function.timeout_duration_),
      buffer_horizon_(function.buffer_horizon_)
{
}

template <typename T> BufferedFunction<T>::~BufferedFunction() {}

template <typename T> ros::Time BufferedFunction<T>::getStartTimestamp() const
{
  return start_timestamp_;
}

template <typename T>
ros::Duration BufferedFunction<T>::getTimeoutDuration() const
{
  return timeout_duration_;
}

template <typename T>
ros::Duration BufferedFunction<T>::getBufferHorizon() const
{
  return buffer_horizon_;
}

template <typename T> bool BufferedFunction<T>::expires() const
{
  return !timeout_duration_.isZero();
}

template <typename T> T BufferedFunction<T>::getValue(double d) const
{
  double q(0.0);
  for (const_iterator it(functions_.begin()); it != functions_.end(); it++)
  {
    FunctionPtr function(*it);
    if (function->getDf() >= d)
    {
      if (function->isNegated())
      {
        q -= function->getValue(d);
      }
      else
      {
        q += function->getValue(d);
      }
    }
  }
  return q;
}

template <typename T>
T BufferedFunction<T>::getValue(const ros::Time& timestamp)
{
  double d((timestamp - start_timestamp_).toSec());
  cleanBuffer(d);
  return getValue(d);
}

template <typename T>
void BufferedFunction<T>::update(const EventConstPtr& event)
{
  update(event->getTimestamp(), model_);
}

template <typename T>
void BufferedFunction<T>::update(const EventConstPtr& event,
                                 const FunctionPtr& model)
{
  update(event->getTimestamp(), model);
  model_ = model;
}

template <typename T>
void BufferedFunction<T>::update(const ros::Time& timestamp)
{
  update(timestamp, model_);
}

template <typename T>
void BufferedFunction<T>::update(const ros::Time& timestamp,
                                 const FunctionPtr& model)
{
  if (timestamp <= last_update_timestamp_)
  {
    ROS_WARN_STREAM("Ignoring already past event in " << *this << ".");
    return;
  }
  if (timestamp < start_timestamp_)
  {
    if (expires() && start_timestamp_ - timestamp > timeout_duration_)
    {
      ROS_WARN_STREAM("Ignoring already past event in " << *this << ".");
      return;
    }
    last_update_timestamp_ = start_timestamp_;
  }
  else
  {
    last_update_timestamp_ = timestamp;
  }
  ros::Duration d(last_update_timestamp_ - start_timestamp_);
  FunctionPtr function;
  if (functions_.empty())
  {
    function.reset(model->clone());
    function->setD0(d);
    if (expires())
    {
      function->setDf(timeout_duration_ + d);
    }
    functions_.push_back(function);
    return;
  }
  function = functions_.back();
  if (!expires())
  {
    if (function->getQf() != model->getQf())
    {
      function->setDf(d);
      function.reset(model->clone());
      function->setD0(d);
      functions_.push_back(function);
    }
    return;
  }
  if (function->getDf() < d.toSec())
  {
    function.reset(model->clone());
    function->setD0(d);
    function->setDf(timeout_duration_ + d);
    functions_.push_back(function);
  }
  else if (function->getQf() != model->getQf())
  {
    function->setDf(d);
    function.reset(model->clone());
    function->setD0(d);
    functions_.push_back(function);
  }
  else
  {
    function->setDf(timeout_duration_ + d);
  }
}

template <typename T>
ros::Time BufferedFunction<T>::getLastUpdateTimestamp() const
{
  return last_update_timestamp_;
}

template <typename T>
void BufferedFunction<T>::setTimeoutDuration(
    const ros::Duration& timeout_duration)
{
  timeout_duration_ = timeout_duration;
}

template <typename T>
void BufferedFunction<T>::setBufferHorizon(const ros::Duration& buffer_horizon)
{
  buffer_horizon_ = buffer_horizon;
}

template <typename T> void BufferedFunction<T>::cleanBuffer(double d)
{
  if (functions_.empty())
  {
    return;
  }
  d -= buffer_horizon_.toSec();
  for (iterator it(functions_.begin()); it != functions_.end();
       it = functions_.erase(it))
  {
    FunctionPtr function(*it);
    if (function->getDf() > d)
    {
      return;
    }
  }
}
}
}

#endif // _UTILITIES_BUFFERED_FUNCTION_H_
