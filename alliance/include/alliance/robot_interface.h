#ifndef _ALLIANCE_ROBOT_INTEFACE_H_
#define _ALLIANCE_ROBOT_INTEFACE_H_

#include "alliance/task.h"
#include <list>
#include <ros/common.h>
#include <utilities/exception.h>
#include <utilities/has_name.h>

namespace alliance
{
template <typename BS> class RobotInterface : public utilities::HasName
{
private:
  typedef boost::shared_ptr<BS> BSPtr;

public:
  typedef typename std::list<BSPtr>::iterator iterator;
  typedef typename std::list<BSPtr>::const_iterator const_iterator;
  RobotInterface(const std::string& id, const std::string& name,
                 const std::string& ns);
  virtual ~RobotInterface();
  void process();
  std::string getNamespace() const;
  TaskPtr getExecutingTask() const;
  ros::Duration getExecutingTaskExpectedDuration() const;
  bool isIdle() const;
  virtual void addBehaviourSet(const BSPtr& behaviour_set);
  std::size_t size() const;
  bool empty() const;
  iterator begin();
  const_iterator begin() const;
  iterator end();
  const_iterator end() const;

protected:
  std::string ns_;
  BSPtr active_behaviour_set_;
  std::list<BSPtr> behaviour_sets_;
  bool contains(const BS& behaviour_set) const;
};

template <typename BS>
RobotInterface<BS>::RobotInterface(const std::string& id,
                                   const std::string& name,
                                   const std::string& ns)
    : HasName::HasName(name, id), ns_(ns)
{
}

template <typename BS> RobotInterface<BS>::~RobotInterface() {}

template <typename BS> void RobotInterface<BS>::process()
{
  if (active_behaviour_set_ && !active_behaviour_set_->isActive())
  {
    active_behaviour_set_->setActive(false);
    active_behaviour_set_.reset();
  }
  for (iterator it(behaviour_sets_.begin()); it != behaviour_sets_.end(); it++)
  {
    BSPtr behaviour_set(*it);
    behaviour_set->preProcess();
    if (behaviour_set->isActive())
    {
      if (active_behaviour_set_ && *active_behaviour_set_ != *behaviour_set)
      {
        active_behaviour_set_->setActive(false);
      }
      active_behaviour_set_ = behaviour_set;
      active_behaviour_set_->process();
    }
  }
}

template <typename BS> std::string RobotInterface<BS>::getNamespace() const
{
  return ns_;
}

template <typename BS> TaskPtr RobotInterface<BS>::getExecutingTask() const
{
  return active_behaviour_set_ ? active_behaviour_set_->getTask() : TaskPtr();
}

template <typename BS>
ros::Duration RobotInterface<BS>::getExecutingTaskExpectedDuration() const
{
  return active_behaviour_set_
             ? active_behaviour_set_->getTaskExpectedDuration()
             : ros::Duration();
}

template <typename BS> bool RobotInterface<BS>::isIdle() const
{
  return !active_behaviour_set_;
}

template <typename BS>
void RobotInterface<BS>::addBehaviourSet(const BSPtr& behaviour_set)
{
  if (!behaviour_set)
  {
    ROS_ERROR_STREAM("The given robot's behaviour set must not be null.");
    return;
  }
  if (contains(*behaviour_set))
  {
    ROS_WARN_STREAM(*this << " robot already have the " << *behaviour_set
                          << " behaviour set.");
    return;
  }
  behaviour_sets_.push_back(behaviour_set);
}

template <typename BS> std::size_t RobotInterface<BS>::size() const
{
  return behaviour_sets_.size();
}

template <typename BS> bool RobotInterface<BS>::empty() const
{
  return behaviour_sets_.empty();
}

template <typename BS>
typename RobotInterface<BS>::iterator RobotInterface<BS>::begin()
{
  return behaviour_sets_.begin();
}

template <typename BS>
typename RobotInterface<BS>::const_iterator RobotInterface<BS>::begin() const
{
  return behaviour_sets_.begin();
}

template <typename BS>
typename RobotInterface<BS>::iterator RobotInterface<BS>::end()
{
  return behaviour_sets_.end();
}

template <typename BS>
typename RobotInterface<BS>::const_iterator RobotInterface<BS>::end() const
{
  return behaviour_sets_.end();
}

template <typename BS>
bool RobotInterface<BS>::contains(const BS& behaviour_set) const
{
  for (const_iterator it(behaviour_sets_.begin()); it != behaviour_sets_.end();
       it++)
  {
    if (**it == behaviour_set)
    {
      return true;
    }
  }
  return false;
}
}

#endif // _ALLIANCE_ROBOT_INTEFACE_H_
