#ifndef _ALLIANCE_ACQUIESCENCE_H_
#define _ALLIANCE_ACQUIESCENCE_H_

#include "alliance/inter_robot_communication.h"
#include <ros/time.h>
#include <utilities/functions/sample_holder.h>

namespace alliance
{
class Acquiescence
{
public:
  Acquiescence(const RobotPtr& robot, const BehaviourSetPtr& behaviour_set);
  virtual ~Acquiescence();
  void init(const InterRobotCommunicationPtr& monitor);
  ros::Duration
  getYieldingDelay(const ros::Time& timestamp = ros::Time::now()) const;
  ros::Duration
  getGivingUpDelay(const ros::Time& timestamp = ros::Time::now()) const;
  bool isAcquiescent(const ros::Time& timestamp = ros::Time::now());
  void setYieldingDelay(const ros::Duration& yielding_delay,
                        const ros::Time& timestamp = ros::Time::now());
  void setGivingUpDelay(const ros::Duration& giving_up_delay,
                        const ros::Time& timestamp = ros::Time::now());

private:
  typedef utilities::functions::ContinuousSampleHolder SampleHolder;
  typedef utilities::functions::ContinuousSampleHolderPtr SampleHolderPtr;
  const RobotPtr robot_;
  const BehaviourSetPtr behaviour_set_;
  InterRobotCommunicationPtr monitor_;
  SampleHolderPtr yielding_delay_;
  SampleHolderPtr giving_up_delay_;
};

typedef boost::shared_ptr<Acquiescence> AcquiescencePtr;
typedef boost::shared_ptr<Acquiescence const> AcquiescenceConstPtr;
}

#endif // _ALLIANCE_ACQUIESCENCE_H_
