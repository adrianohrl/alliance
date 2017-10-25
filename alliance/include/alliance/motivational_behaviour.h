#ifndef _ALLIANCE_MOTIVATIONAL_BEHAVIOUR_H_
#define _ALLIANCE_MOTIVATIONAL_BEHAVIOUR_H_

#include <ros/time.h>
#include "alliance/acquiescence.h"
#include "alliance/activity_suppression.h"
#include "alliance/impatience.h"
#include "alliance/impatience_reset.h"
#include "alliance/inter_robot_communication.h"
#include "alliance/sensory_feedback.h"
#include <alliance_msgs/Motivation.h>
#include <utilities/functions/sample_holder.h>
#include <utilities/ros_message_converter.h>

namespace alliance
{
class MotivationalBehaviour
    : public utilities::ROSMessageConverter<alliance_msgs::Motivation>
{
public:
  MotivationalBehaviour(const RobotPtr& robot,
                        const BehaviourSetPtr& behaviour_set);
  virtual ~MotivationalBehaviour();
  void init();
  bool isActive(const ros::Time& timestamp = ros::Time::now());
  double getThreshold(const ros::Time& timestamp = ros::Time::now()) const;
  double getLevel(const ros::Time& timestamp = ros::Time::now());
  ActivitySuppressionPtr getActivitySuppression() const;
  ImpatiencePtr getImpatience() const;
  ImpatienceResetPtr getImpatienceReset() const;
  InterRobotCommunicationPtr getInterRobotCommunication() const;
  SensoryFeedbackPtr getSensoryFeedback() const;
  void setThreshold(double threshold,
                    const ros::Time& timestamp = ros::Time::now());
  void setImpatience(double fast_rate,
                     const ros::Time& timestamp = ros::Time::now());
  void setAcquiescence(const ros::Duration& yielding_delay,
                       const ros::Duration& giving_up_delay,
                       const ros::Time& timestamp = ros::Time::now());
  virtual bool operator==(const alliance_msgs::Motivation& msg) const;

private:
  typedef utilities::functions::ContinuousSampleHolder SampleHolder;
  typedef utilities::functions::ContinuousSampleHolderPtr SampleHolderPtr;
  const RobotPtr robot_;
  const BehaviourSetPtr behaviour_set_;
  AcquiescencePtr acquiescence_;
  ActivitySuppressionPtr activity_suppression_;
  ImpatiencePtr impatience_;
  ImpatienceResetPtr impatience_reset_;
  InterRobotCommunicationPtr monitor_;
  SensoryFeedbackPtr sensory_feedback_;
  SampleHolderPtr threshold_;
  SampleHolderPtr motivation_;
};

typedef boost::shared_ptr<MotivationalBehaviour> MotivationalBehaviourPtr;
typedef boost::shared_ptr<MotivationalBehaviour const>
    MotivationalBehaviourConstPtr;
}

#endif // _ALLIANCE_MOTIVATIONAL_BEHAVIOUR_H_
