#ifndef _UTILITIES_SAMPLE_HOLDER_H_
#define _UTILITIES_SAMPLE_HOLDER_H_

#include "utilities/functions/buffered_function.h"
#include "utilities/functions/step_function.h"
#include "utilities/functions/value_change_event.h"

namespace utilities
{
namespace functions
{
template <typename T> class SampleHolder : public BufferedFunction<T>
{
protected:
  typedef typename StepFunction<T>::Ptr StepFunctionPtr;
  typedef typename StepFunction<T>::ConstPtr StepFunctionConstPtr;

public:
  typedef boost::shared_ptr<SampleHolder<T> > Ptr;
  typedef boost::shared_ptr<SampleHolder<T> const> ConstPtr;
  SampleHolder(const std::string& id, const T& value,
               const ros::Duration& buffer_horizon,
               const ros::Time& start_timestamp = ros::Time::now());
  SampleHolder(const std::string& id, const StepFunctionPtr& model,
               const ros::Duration& buffer_horizon,
               const ros::Time& start_timestamp = ros::Time::now());
  SampleHolder(const std::string& id, const T& value,
               const ros::Duration& timeout_duration,
               const ros::Duration& buffer_horizon,
               const ros::Time& start_timestamp = ros::Time::now());
  SampleHolder(const std::string& id, const StepFunctionPtr& model,
               const ros::Duration& timeout_duration,
               const ros::Duration& buffer_horizon,
               const ros::Time& start_timestamp = ros::Time::now());
  SampleHolder(const SampleHolder<T>& sample_holder);
  virtual ~SampleHolder();
  void update(const typename ValueChangeEvent<T>::ConstPtr& event);
  virtual void update(const ros::Time& timestamp);
  void update(const T& value, const ros::Time& timestamp);
};

typedef SampleHolder<utilities::ContinuousSignalType> ContinuousSampleHolder;
typedef boost::shared_ptr<ContinuousSampleHolder> ContinuousSampleHolderPtr;
typedef boost::shared_ptr<ContinuousSampleHolder const>
    ContinuousSampleHolderConstPtr;
typedef SampleHolder<utilities::DiscreteSignalType> DiscreteSampleHolder;
typedef boost::shared_ptr<DiscreteSampleHolder> DiscreteSampleHolderPtr;
typedef boost::shared_ptr<DiscreteSampleHolder const>
    DiscreteSampleHolderConstPtr;

template <typename T>
SampleHolder<T>::SampleHolder(const std::string& id, const T& value,
                              const ros::Duration& buffer_horizon,
                              const ros::Time& start_timestamp)
    : BufferedFunction<T>::BufferedFunction(
          id, typename StepFunction<T>::Ptr(new StepFunction<T>(value, true)),
          buffer_horizon, start_timestamp)
{
}

template <typename T>
SampleHolder<T>::SampleHolder(const std::string& id,
                              const StepFunctionPtr& model,
                              const ros::Duration& buffer_horizon,
                              const ros::Time& start_timestamp)
    : BufferedFunction<T>::BufferedFunction(id, model, buffer_horizon,
                                            start_timestamp)
{
}

template <typename T>
SampleHolder<T>::SampleHolder(const std::string& id, const T& value,
                              const ros::Duration& timeout_duration,
                              const ros::Duration& buffer_horizon,
                              const ros::Time& start_timestamp)
    : BufferedFunction<T>::BufferedFunction(
          id, StepFunction<T>::Ptr(new StepFunction<T>(value, true)),
          timeout_duration, buffer_horizon, start_timestamp)
{
}

template <typename T>
SampleHolder<T>::SampleHolder(const std::string& id,
                              const StepFunctionPtr& model,
                              const ros::Duration& timeout_duration,
                              const ros::Duration& buffer_horizon,
                              const ros::Time& start_timestamp)
    : BufferedFunction<T>::BufferedFunction(id, model, timeout_duration,
                                            buffer_horizon, start_timestamp)
{
}

template <typename T>
SampleHolder<T>::SampleHolder(const SampleHolder<T>& sample_holder)
    : BufferedFunction<T>::BufferedFunction(sample_holder)
{
}

template <typename T> SampleHolder<T>::~SampleHolder() {}

template <typename T>
void SampleHolder<T>::update(
    const typename ValueChangeEvent<T>::ConstPtr& event)
{
  BufferedFunction<T>::update(event,
                              new StepFunction<T>(event->getValue(), true));
}

template <typename T> void SampleHolder<T>::update(const ros::Time& timestamp)
{
  BufferedFunction<T>::update(timestamp);
}

template <typename T>
void SampleHolder<T>::update(const T& value, const ros::Time& timestamp)
{
  BufferedFunction<T>::update(
      timestamp, StepFunctionPtr(new StepFunction<T>(value, true, false)));
}
}
}

#endif // _UTILITIES_SAMPLE_HOLDER_H_
