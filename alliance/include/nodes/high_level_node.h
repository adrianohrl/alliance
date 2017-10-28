#ifndef _NODES_ALLIANCE_HIGH_LEVEL_NODE_H_
#define _NODES_ALLIANCE_HIGH_LEVEL_NODE_H_

#include "alliance/robot.h"
#include "nodes/alliance_node.h"
#include "nodes/motivation_publishers.h"

namespace nodes
{
class HighLevelNode
    : public nodes::AllianceNode<alliance::Robot, HighLevelNode>,
      public SensoryFeedbackSubject
{
public:
  HighLevelNode(const ros::NodeHandlePtr& nh,
                const ros::Rate& rate = ros::Rate(30.0));
  virtual ~HighLevelNode();

private:
  bool broadcasting_;
  ros::Publisher inter_robot_communication_pub_;
  MotivationPublishersPtr motivation_pubs_;
  ros::Subscriber sensory_feedback_sub_;
  ros::Timer broadcast_timer_;
  virtual void readParameters();
  virtual void init();
  virtual void controlLoop();
  void broadcastTimerCallback(const ros::TimerEvent& event);
  void sensoryFeedbackCallback(const alliance_msgs::SensoryFeedback& msg);
};

typedef boost::shared_ptr<HighLevelNode> HighLevelNodePtr;
}

#endif // _NODES_ALLIANCE_HIGH_LEVEL_NODE_H_
