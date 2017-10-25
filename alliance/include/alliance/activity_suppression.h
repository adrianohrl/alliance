#ifndef _ALLIANCE_ACTIVITY_SUPPRESSION_H_
#define _ALLIANCE_ACTIVITY_SUPPRESSION_H_

#include <ros/time.h>
#include <utilities/observer.h>
#include <utilities/functions/unary_sample_holder.h>

namespace alliance
{
class Robot;
typedef boost::shared_ptr<Robot> RobotPtr;
typedef boost::shared_ptr<Robot const> RobotConstPtr;

class BehaviourSet;
typedef boost::shared_ptr<BehaviourSet> BehaviourSetPtr;
typedef boost::shared_ptr<BehaviourSet const> BehaviourSetConstPtr;

class ActivitySuppression : public utilities::Observer
{
public:
  ActivitySuppression(const RobotPtr& robot,
                      const BehaviourSetPtr& behaviour_set);
  virtual ~ActivitySuppression();
  virtual void update(const utilities::EventConstPtr& event);
  bool isSuppressed(const ros::Time& timestamp = ros::Time::now()) const;

private:
  typedef utilities::functions::UnarySampleHolder SampleHolder;
  typedef utilities::functions::UnarySampleHolderPtr SampleHolderPtr;
  const RobotPtr robot_;
  const BehaviourSetPtr behaviour_set_;
  SampleHolderPtr suppressed_;
};

typedef boost::shared_ptr<ActivitySuppression> ActivitySuppressionPtr;
typedef boost::shared_ptr<ActivitySuppression const>
    ActivitySuppressionConstPtr;
}

#endif // _ALLIANCE_ACTIVITY_SUPPRESSION_H_
