#ifndef _UTILITIES_UNARY_SAMPLE_HOLDER_H_
#define _UTILITIES_UNARY_SAMPLE_HOLDER_H_

#include "utilities/unary_signal_type.h"
#include "utilities/functions/sample_holder.h"
#include "utilities/toggle_event.h"

namespace utilities
{
namespace functions
{
class UnarySampleHolder : public SampleHolder<UnarySignalType>
{
public:
  UnarySampleHolder(const std::string& id, const ros::Duration& buffer_horizon,
                    const ros::Time& start_timestamp = ros::Time::now());
  UnarySampleHolder(const std::string& id,
                    const ros::Duration& timeout_duration,
                    const ros::Duration& buffer_horizon,
                    const ros::Time& start_timestamp = ros::Time::now());
  UnarySampleHolder(const UnarySampleHolder& function);
  virtual ~UnarySampleHolder();
  using BufferedFunction<UnarySignalType>::update;
  using SampleHolder<UnarySignalType>::update;
  void update(const ToggleEventConstPtr& event);
  bool updated(const ros::Time& t1,
               const ros::Time& t2 = ros::Time::now()) const;
};

typedef boost::shared_ptr<UnarySampleHolder> UnarySampleHolderPtr;
typedef boost::shared_ptr<UnarySampleHolder const> UnarySampleHolderConstPtr;
}
}

#endif // _UTILITIES_UNARY_SAMPLE_HOLDER_H_
