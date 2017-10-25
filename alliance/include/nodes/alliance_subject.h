#ifndef _UTILITIES_ALLIANCE_SUBJECT_H_
#define _UTILITIES_ALLIANCE_SUBJECT_H_

#include "nodes/alliance_observer.h"
#include <utilities/subject.h>

namespace nodes
{
template <typename M> class AllianceSubject : public utilities::Subject
{
private:
  typedef typename AllianceEvent<M>::Ptr AllianceEventPtr;
  typedef typename AllianceEvent<M>::ConstPtr AllianceEventConstPtr;
  typedef typename AllianceObserver<M>::Ptr AllianceObserverPtr;
  typedef typename AllianceObserver<M>::ConstPtr AllianceObserverConstPtr;

public:
  typedef boost::shared_ptr<AllianceSubject<M> > Ptr;
  typedef boost::shared_ptr<AllianceSubject<M> const> ConstPtr;
  AllianceSubject(const std::string& id);
  AllianceSubject(const AllianceSubject<M>& subject);
  virtual ~AllianceSubject();
  void notify(const AllianceEventConstPtr& event);
  void registerObserver(const AllianceObserverPtr& observer);
};

typedef AllianceSubject<alliance_msgs::InterRobotCommunication>
    InterRobotCommunicationSubject;
typedef AllianceSubject<alliance_msgs::InterRobotCommunication>::Ptr
    InterRobotCommunicationSubjectPtr;
typedef AllianceSubject<alliance_msgs::InterRobotCommunication>::ConstPtr
    InterRobotCommunicationSubjectConstPtr;
typedef AllianceSubject<alliance_msgs::SensoryFeedback> SensoryFeedbackSubject;
typedef AllianceSubject<alliance_msgs::SensoryFeedback>::Ptr
    SensoryFeedbackSubjectPtr;
typedef AllianceSubject<alliance_msgs::SensoryFeedback>::ConstPtr
    SensoryFeedbackSubjectConstPtr;

template <typename M>
AllianceSubject<M>::AllianceSubject(const std::string& id)
    : Subject::Subject(id)
{
}

template <typename M>
AllianceSubject<M>::AllianceSubject(const AllianceSubject<M>& subject)
    : Subject::Subject(subject)
{
}

template <typename M> AllianceSubject<M>::~AllianceSubject() {}

template <typename M>
void AllianceSubject<M>::notify(const AllianceEventConstPtr& event)
{
  Subject::notify(event);
}

template <typename M>
void AllianceSubject<M>::registerObserver(const AllianceObserverPtr& observer)
{
  Subject::registerObserver(observer);
}
}

#endif // _UTILITIES_ALLIANCE_SUBJECT_H_
