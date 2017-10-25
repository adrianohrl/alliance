#ifndef _ALLIANCE_IMPATIENCE_H_
#define _ALLIANCE_IMPATIENCE_H_

#include "alliance/inter_robot_communication.h"
#include <map>
#include <ros/time.h>
#include <utilities/functions/sample_holder.h>

namespace alliance
{
class Impatience : public nodes::InterRobotCommunicationObserver
{
public:
  Impatience(const RobotPtr& robot, const BehaviourSetPtr& behaviour_set);
  virtual ~Impatience();
  void init(const InterRobotCommunicationPtr& monitor);
  double getSlowRate(const std::string& robot_id,
                     const ros::Time& timestamp = ros::Time::now()) const;
  double getFastRate(const ros::Time& timestamp) const;
  ros::Duration
  getReliabilityDuration(const std::string& robot_id,
                         const ros::Time& timestamp = ros::Time::now()) const;
  double getLevel(const ros::Time& timestamp = ros::Time::now()) const;
  void setSlowRate(const std::string& robot_id, double slow_rate,
                   const ros::Time& timestamp = ros::Time::now());
  void setFastRate(double fast_rate,
                   const ros::Time& timestamp = ros::Time::now());
  void setReliabilityDuration(const std::string& robot_id,
                              const ros::Duration& reliability_duration,
                              const ros::Time& timestamp = ros::Time::now());
  virtual void update(const nodes::InterRobotCommunicationEventConstPtr& event);

private:
  typedef utilities::functions::ContinuousSampleHolder SampleHolder;
  typedef utilities::functions::ContinuousSampleHolderPtr SampleHolderPtr;
  typedef std::map<std::string, SampleHolderPtr>::iterator iterator;
  typedef std::map<std::string, SampleHolderPtr>::const_iterator const_iterator;
  const RobotPtr robot_;
  const BehaviourSetPtr behaviour_set_;
  InterRobotCommunicationPtr monitor_;
  std::map<std::string, SampleHolderPtr> slow_rates_;
  SampleHolderPtr fast_rate_;
  std::map<std::string, SampleHolderPtr> reliability_durations_;
};

typedef boost::shared_ptr<Impatience> ImpatiencePtr;
typedef boost::shared_ptr<Impatience const> ImpatienceConstPtr;
}

#endif // _ALLIANCE_IMPATIENCE_H_
