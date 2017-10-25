#ifndef _NODES_ALLIANCE_LOW_LEVEL_NODE_H_
#define _NODES_ALLIANCE_LOW_LEVEL_NODE_H_

#include "alliance/behaved_robot.h"
#include "nodes/alliance_node.h"

namespace nodes
{
class LowLevelNode
    : public nodes::AllianceNode<alliance::BehavedRobot, LowLevelNode>
{
public:
  LowLevelNode(const ros::NodeHandlePtr& nh,
               const ros::Rate& rate = ros::Rate(30.0));
  virtual ~LowLevelNode();

private:
  virtual void readParameters();
  virtual void init();
};

typedef boost::shared_ptr<LowLevelNode> LowLevelNodePtr;
}

#endif // _NODES_ALLIANCE_LOW_LEVEL_NODE_H_
