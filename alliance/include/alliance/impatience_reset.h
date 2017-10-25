#ifndef _ALLIANCE_IMPATIENCE_RESET_H_
#define _ALLIANCE_IMPATIENCE_RESET_H_

#include "alliance/inter_robot_communication.h"
#include <ros/time.h>
#include <utilities/observer.h>

namespace alliance
{
class ImpatienceReset
{
public:
  ImpatienceReset();
  virtual ~ImpatienceReset();
  void init(const InterRobotCommunicationPtr& monitor);
  bool isResetted(const ros::Time& timestamp = ros::Time::now());

private:
  bool resetted_;
  InterRobotCommunicationPtr monitor_;
};

typedef boost::shared_ptr<ImpatienceReset> ImpatienceResetPtr;
typedef boost::shared_ptr<ImpatienceReset const> ImpatienceResetConstPtr;
}

#endif // _ALLIANCE_IMPATIENCE_RESET_H_
