#ifndef _ALLIANCE_BEHAVIOUR_SET_H_
#define _ALLIANCE_BEHAVIOUR_SET_H_

#include <boost/enable_shared_from_this.hpp>
#include "alliance/behaviour_set_interface.h"
#include "alliance/motivational_behaviour.h"
#include <list>

namespace alliance
{
class BehaviourSet : public BehaviourSetInterface<Robot>,
                     public utilities::Subject,
                     public boost::enable_shared_from_this<BehaviourSet>
{
public:
  BehaviourSet(const RobotPtr& robot, const TaskPtr& task,
               const ros::Duration& buffer_horizon);
  virtual ~BehaviourSet();
  void init();
  virtual void preProcess();
  MotivationalBehaviourPtr getMotivationalBehaviour() const;
  void setActivationThreshold(double threshold);
  void setAcquiescence(const ros::Duration& yielding_delay,
                       const ros::Duration& giving_up_delay);
  void setImpatience(double fast_rate);
  void registerActivitySuppression(const BehaviourSetPtr& behaviour_set);
  virtual void setActive(bool active = true,
                         const ros::Time& timestamp = ros::Time::now());

private:
  const RobotPtr robot_;
  MotivationalBehaviourPtr motivational_behaviour_;
};

typedef boost::shared_ptr<BehaviourSet> BehaviourSetPtr;
typedef boost::shared_ptr<BehaviourSet const> BehaviourSetConstPtr;
}

#endif // _ALLIANCE_BEHAVIOUR_SET_H_
