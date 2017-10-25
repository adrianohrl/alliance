#ifndef _ALLIANCE_ROBOT_H_
#define _ALLIANCE_ROBOT_H_

#include <list>
#include <ros/time.h>
#include <ros/rate.h>
#include "alliance/behaviour_set.h"
#include "alliance/robot_interface.h"

namespace alliance
{
class Robot : public RobotInterface<BehaviourSet>
{
public:
  Robot(const std::string& id, const std::string& name, const std::string& ns);
  virtual ~Robot();
  ros::Rate getBroadcastRate() const;
  ros::Duration getTimeoutDuration() const;
  void setBroadcastRate(const ros::Rate& broadcast_rate);
  void setTimeoutDuration(const ros::Duration& timeout_duration);
  virtual void addBehaviourSet(const BehaviourSetPtr& behaviour_set);

private:
  ros::Rate broadcast_rate_;
  ros::Duration timeout_duration_;
};

typedef boost::shared_ptr<Robot> RobotPtr;
typedef boost::shared_ptr<Robot const> RobotConstPtr;
}

#endif // _ALLIANCE_ROBOT_H_
