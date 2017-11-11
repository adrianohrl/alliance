#ifndef _UTILITIES_TOGGLE_EVENT_H_
#define _UTILITIES_TOGGLE_EVENT_H_

#include "utilities/event.h"

namespace utilities
{
class ToggleEvent : public Event
{
public:
  ToggleEvent(const SubjectPtr& subject, bool value,
              const ros::Time& timestamp = ros::Time::now());
  ToggleEvent(const ToggleEvent& event);
  virtual ~ToggleEvent();
  bool getValue() const;

private:
  bool value_;
};

typedef boost::shared_ptr<ToggleEvent> ToggleEventPtr;
typedef boost::shared_ptr<ToggleEvent const> ToggleEventConstPtr;
}

#endif // _UTILITIES_TOGGLE_EVENT_H_
