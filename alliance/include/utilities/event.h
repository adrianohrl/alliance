/**
 *  This header file defines and implements the Event class of
 *the Observer Design Pattern.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_EVENT_H_
#define _UTILITIES_EVENT_H_

#include <ros/time.h>

namespace utilities
{
class Subject;
typedef boost::shared_ptr<Subject> SubjectPtr;

class Event
{
public:
  Event(const SubjectPtr& subject,
        const ros::Time& timestamp = ros::Time::now());
  Event(const Event& event);
  virtual ~Event();
  ros::Time getTimestamp() const;
  SubjectPtr getSubject() const;

protected:
  const SubjectPtr subject_;

private:
  const ros::Time timestamp_;
};

typedef boost::shared_ptr<Event> EventPtr;
typedef boost::shared_ptr<Event const> EventConstPtr;
}

#endif // _UTILITIES_EVENT_H_
