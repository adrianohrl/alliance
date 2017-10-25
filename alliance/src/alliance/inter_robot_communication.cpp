#include "alliance/inter_robot_communication.h"
#include "alliance/behaviour_set.h"
#include "alliance/robot.h"

namespace alliance
{
InterRobotCommunication::InterRobotCommunication(
    const RobotPtr& robot, const BehaviourSetPtr& behaviour_set)
    : AllianceObserver<alliance_msgs::InterRobotCommunication>::
          AllianceObserver(behaviour_set->getId() + "/inter_communication"),
      robot_(robot), behaviour_set_(behaviour_set),
      last_update_timestamp_(ros::Time::now())
{
}

InterRobotCommunication::~InterRobotCommunication() {}

bool InterRobotCommunication::received(const ros::Time& t1,
                                       const ros::Time& t2) const
{
  for (const_iterator it(robots_.begin()); it != robots_.end(); it++)
  {
    SampleHolderPtr sample_holder(it->second);
    if (sample_holder->updated(t1, t2))
    {
      return true;
    }
  }
  return false;
}

bool InterRobotCommunication::received(const std::string& robot_id,
                                       const ros::Time& t1,
                                       const ros::Time& t2) const
{
  const_iterator it(robots_.find(robot_id));
  if (it == robots_.end())
  {
    return false;
  }
  SampleHolderPtr sample_holder(it->second);
  return sample_holder->updated(t1, t2);
}

void InterRobotCommunication::update(
    const nodes::InterRobotCommunicationEventConstPtr& event)
{
  if (event->isRelated(*robot_) ||
      !event->isRelated(*behaviour_set_->getTask()))
  {
    return;
  }
  std::string robot_id(event->getMsg().header.frame_id);
  iterator it(robots_.find(robot_id));
  SampleHolderPtr sample_holder;
  if (it == robots_.end())
  {
    sample_holder.reset(new SampleHolder(
        getId() + "/" + robot_id + "/sample_holder",
        robot_->getTimeoutDuration(), behaviour_set_->getBufferHorizon(),
        event->getTimestamp()));
    robots_[robot_id] = sample_holder;
  }
  else
  {
    sample_holder = it->second;
  }
  ROS_DEBUG_STREAM("Updating " << *sample_holder << ".");
  sample_holder->update(event->getTimestamp());
  last_update_timestamp_ = event->getTimestamp();
}

ros::Time InterRobotCommunication::getLastUpdateTimestamp() const
{
  return last_update_timestamp_;
}
}
