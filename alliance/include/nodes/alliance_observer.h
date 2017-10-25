#ifndef _UTILITIES_ALLIANCE_OBSERVER_H_
#define _UTILITIES_ALLIANCE_OBSERVER_H_

#include <utilities/observer.h>

namespace nodes
{
template <typename M> class AllianceEvent;

template <typename M> class AllianceObserver : public utilities::Observer
{
private:
  typedef typename AllianceEvent<M>::Ptr AllianceEventPtr;
  typedef typename AllianceEvent<M>::ConstPtr AllianceEventConstPtr;

public:
  typedef boost::shared_ptr<AllianceObserver<M> > Ptr;
  typedef boost::shared_ptr<AllianceObserver const> ConstPtr;
  AllianceObserver(const std::string& id);
  AllianceObserver(const AllianceObserver<M>& observer);
  virtual ~AllianceObserver();
  virtual void update(const utilities::EventConstPtr& event);
  virtual void update(const AllianceEventConstPtr& event) = 0;
};
}

#include "nodes/alliance_event.h"

namespace nodes
{
typedef AllianceObserver<alliance_msgs::InterRobotCommunication>
    InterRobotCommunicationObserver;
typedef AllianceObserver<alliance_msgs::InterRobotCommunication>::Ptr
    InterRobotCommunicationObserverPtr;
typedef AllianceObserver<alliance_msgs::InterRobotCommunication>::ConstPtr
    InterRobotCommunicationObserverConstPtr;
typedef AllianceObserver<alliance_msgs::SensoryFeedback> SensoryFeedbackObserver;
typedef AllianceObserver<alliance_msgs::SensoryFeedback>::Ptr
    SensoryFeedbackObserverPtr;
typedef AllianceObserver<alliance_msgs::SensoryFeedback>::ConstPtr
    SensoryFeedbackObserverConstPtr;

template <typename M>
AllianceObserver<M>::AllianceObserver(const std::string& id)
    : Observer::Observer(id)
{
}

template <typename M>
AllianceObserver<M>::AllianceObserver(const AllianceObserver<M>& observer)
    : Observer::Observer(observer)
{
}

template <typename M> AllianceObserver<M>::~AllianceObserver() {}

template <typename M>
void AllianceObserver<M>::update(const utilities::EventConstPtr& event)
{
  AllianceEventConstPtr alliance_event(
      boost::dynamic_pointer_cast<AllianceEvent<M> const>(event));
  if (alliance_event)
  {
    update(alliance_event);
  }
}
}

#endif // _UTILITIES_ALLIANCE_OBSERVER_H_
