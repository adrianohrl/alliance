/**
 *  This header file defines and implements the Subject class of the Observer
 *Design Pattern.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_SUBJECT_H_
#define _UTILITIES_SUBJECT_H_

#include <list>
#include <ros/common.h>
#include "utilities/observer.h"

namespace utilities
{
class Subject : public HasId<std::string>
{
public:
  virtual ~Subject();

protected:
  Subject(const std::string& id);
  Subject(const Subject& subject);
  void registerObserver(const ObserverPtr& observer);
  void unregisterObserver(const ObserverPtr& observer);
  void clearObservers();
  void notify(const EventConstPtr& event);
  bool empty() const;

private:
  typedef std::list<ObserverPtr>::iterator iterator;
  typedef std::list<ObserverPtr>::const_iterator const_iterator;
  std::list<ObserverPtr> observers_;
};

typedef boost::shared_ptr<Subject> SubjectPtr;
typedef boost::shared_ptr<Subject const> SubjectConstPtr;
}

#endif // _UTILITIES_SUBJECT_H_
