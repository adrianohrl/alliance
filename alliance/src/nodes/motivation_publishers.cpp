#include "nodes/motivation_publishers.h"

namespace nodes
{
MotivationPublishers::MotivationPublishers(const ros::NodeHandlePtr& nh) : nh_(nh)
{
}

MotivationPublishers::~MotivationPublishers() {}

void MotivationPublishers::addMotivationBehaviour(
    const alliance::RobotConstPtr& robot,
    const alliance::BehaviourSetConstPtr& behaviour_set)
{
  std::string topic_name(robot->getId() + "/alliance/motivation/" +
                         behaviour_set->getTask()->getId());
  std::pair<alliance::MotivationalBehaviourPtr, ros::Publisher> pub(
      behaviour_set->getMotivationalBehaviour(),
      nh_->advertise<alliance_msgs::Motivation>(topic_name, 1));
  pubs_.insert(pub);
}

void MotivationPublishers::publish() const
{
  for (const_iterator it(pubs_.begin()); it != pubs_.end(); it++)
  {
    it->second.publish(it->first->toMsg());
  }
}

void MotivationPublishers::shutdown()
{
  for (iterator it(pubs_.begin()); it != pubs_.end(); it++)
  {
    it->second.shutdown();
  }
}
}
