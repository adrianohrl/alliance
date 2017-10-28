#ifndef _NODE_MOTIVATION_PUBLISHER_H_
#define _NODE_MOTIVATION_PUBLISHER_H_

#include "alliance/motivational_behaviour.h"
#include "alliance/robot.h"
#include <alliance_msgs/Motivation.h>
#include <list>
#include <map>
#include <ros/node_handle.h>
#include <ros/publisher.h>

namespace nodes
{
class MotivationPublishers
{
public:
  MotivationPublishers(const ros::NodeHandlePtr& nh);
  virtual ~MotivationPublishers();
  void addMotivationBehaviour(const alliance::RobotConstPtr& robot,
      const alliance::BehaviourSetConstPtr& behaviour_set);
  void publish() const;
  void shutdown();

private:
  typedef std::map<alliance::MotivationalBehaviourPtr, ros::Publisher>::iterator
      iterator;
  typedef std::map<alliance::MotivationalBehaviourPtr,
                   ros::Publisher>::const_iterator const_iterator;
  ros::NodeHandlePtr nh_;
  std::map<alliance::MotivationalBehaviourPtr, ros::Publisher> pubs_;
};

typedef boost::shared_ptr<MotivationPublishers> MotivationPublishersPtr;
typedef boost::shared_ptr<MotivationPublishers const>
    MotivationPublishersConstPtr;
}

#endif // _NODE_MOTIVATION_PUBLISHER_H_
