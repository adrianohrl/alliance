#ifndef _UTILITIES_ALLIANCE_EVENT_H_
#define _UTILITIES_ALLIANCE_EVENT_H_

#include "alliance/task.h"
#include "alliance_msgs/InterRobotCommunication.h"
#include "alliance_msgs/SensoryFeedback.h"
#include <utilities/event.h>

namespace alliance
{
template <typename BS> class RobotInterface;
}

namespace nodes
{
template <typename M> class AllianceSubject;

template <typename M> class AllianceEvent : public utilities::Event
{
private:
  typedef boost::shared_ptr<AllianceSubject<M> > AllianceSubjectPtr;
  typedef boost::shared_ptr<AllianceSubject<M> const> AllianceSubjectConstPtr;

public:
  typedef boost::shared_ptr<AllianceEvent<M> > Ptr;
  typedef boost::shared_ptr<AllianceEvent<M> const> ConstPtr;
  AllianceEvent(const AllianceSubjectPtr& subject, const M& msg);
  AllianceEvent(const AllianceEvent<M>& event);
  virtual ~AllianceEvent();
  M getMsg() const;
  template <typename BS>
  bool isRelated(const alliance::RobotInterface<BS>& robot) const;
  bool isRelated(const alliance::Task& task) const;

private:
  M msg_;
};

typedef AllianceEvent<alliance_msgs::InterRobotCommunication>
    InterRobotCommunicationEvent;
typedef AllianceEvent<alliance_msgs::InterRobotCommunication>::Ptr
    InterRobotCommunicationEventPtr;
typedef AllianceEvent<alliance_msgs::InterRobotCommunication>::ConstPtr
    InterRobotCommunicationEventConstPtr;
typedef AllianceEvent<alliance_msgs::SensoryFeedback> SensoryFeedbackEvent;
typedef AllianceEvent<alliance_msgs::SensoryFeedback>::Ptr SensoryFeedbackEventPtr;
typedef AllianceEvent<alliance_msgs::SensoryFeedback>::ConstPtr
    SensoryFeedbackEventConstPtr;
}

#include "alliance/robot_interface.h"
#include "nodes/alliance_subject.h"

namespace nodes
{
template <typename M>
AllianceEvent<M>::AllianceEvent(const AllianceSubjectPtr& subject, const M& msg)
    : Event::Event(subject, msg.header.stamp), msg_(msg)
{
}

template <typename M>
AllianceEvent<M>::AllianceEvent(const AllianceEvent<M>& event)
    : Event::Event(event)
{
}

template <typename M> AllianceEvent<M>::~AllianceEvent() {}

template <typename M> M AllianceEvent<M>::getMsg() const { return msg_; }

template <typename M>
template <typename BS>
bool AllianceEvent<M>::isRelated(
    const alliance::RobotInterface<BS>& robot) const
{
  return msg_.header.frame_id == robot.getId();
}

template <typename M>
bool AllianceEvent<M>::isRelated(const alliance::Task& task) const
{
  return msg_.task_id == task.getId();
}
}

#endif // _UTILITIES_ALLIANCE_EVENT_H_
