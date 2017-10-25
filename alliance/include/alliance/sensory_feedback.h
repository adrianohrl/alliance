#ifndef _ALLIANCE_SENSORY_FEEDBACK_H_
#define _ALLIANCE_SENSORY_FEEDBACK_H_

#include "alliance/task.h"
#include <list>
#include <ros/time.h>
#include <utilities/functions/unary_sample_holder.h>
#include "nodes/alliance_observer.h"

namespace alliance
{
class Robot;
typedef boost::shared_ptr<Robot> RobotPtr;
typedef boost::shared_ptr<Robot const> RobotConstPtr;

class BehaviourSet;
typedef boost::shared_ptr<BehaviourSet> BehaviourSetPtr;
typedef boost::shared_ptr<BehaviourSet const> BehaviourSetConstPtr;

class SensoryFeedback : public nodes::SensoryFeedbackObserver
{
public:
  SensoryFeedback(const RobotPtr& robot, const BehaviourSetPtr& behaviour_set);
  virtual ~SensoryFeedback();
  virtual void update(const nodes::SensoryFeedbackEventConstPtr &event);
  bool isApplicable(const ros::Time& timestamp = ros::Time::now());

private:
  typedef utilities::functions::UnarySampleHolder SampleHolder;
  typedef utilities::functions::UnarySampleHolderPtr SampleHolderPtr;
  const RobotPtr robot_;
  const BehaviourSetPtr behaviour_set_;
  SampleHolderPtr applicable_;
};

typedef boost::shared_ptr<SensoryFeedback> SensoryFeedbackPtr;
typedef boost::shared_ptr<SensoryFeedback const> SensoryFeedbackConstPtr;
}

#endif // _ALLIANCE_SENSORY_FEEDBACK_H_
