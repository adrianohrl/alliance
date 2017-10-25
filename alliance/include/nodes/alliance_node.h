#ifndef _NODES_ALLIANCE_NODE_H_
#define _NODES_ALLIANCE_NODE_H_

#include <boost/enable_shared_from_this.hpp>
#include "nodes/alliance_subject.h"
#include <utilities/ros_node.h>

namespace nodes
{
template <typename R, typename N>
class AllianceNode : public utilities::ROSNode,
                     public InterRobotCommunicationSubject,
                     public boost::enable_shared_from_this<N>
{
private:
  typedef boost::shared_ptr<R> RPtr;
  typedef boost::shared_ptr<R const> RConstPtr;
  typedef boost::shared_ptr<N> NPtr;
  typedef boost::shared_ptr<N const> NConstPtr;

public:
  AllianceNode(const ros::NodeHandlePtr& nh, const ros::Rate& rate);
  virtual ~AllianceNode();

protected:
  RPtr robot_;
  virtual void init();
  virtual void controlLoop();

private:
  ros::Subscriber inter_robot_communication_sub_;
  void interRobotCommunicationCallback(
      const alliance_msgs::InterRobotCommunication& msg);
};

template <typename R, typename N>
AllianceNode<R, N>::AllianceNode(const ros::NodeHandlePtr& nh,
                                 const ros::Rate& rate)
    : ROSNode::ROSNode(nh, rate),
      AllianceSubject<alliance_msgs::InterRobotCommunication>::AllianceSubject(
          ros::this_node::getName() + "/inter_robot_comunication")
{
  inter_robot_communication_sub_ =
      nh->subscribe("/alliance/inter_robot_communication", 25,
                    &AllianceNode<R, N>::interRobotCommunicationCallback, this);
}

template <typename R, typename N> AllianceNode<R, N>::~AllianceNode()
{
  inter_robot_communication_sub_.shutdown();
}

template <typename R, typename N> void AllianceNode<R, N>::init()
{
  if (!robot_)
  {
    throw utilities::Exception("The " + str() +
                               "'s robot was not created yet!!!");
  }
}

template <typename R, typename N> void AllianceNode<R, N>::controlLoop()
{
  robot_->process();
}

template <typename R, typename N>
void AllianceNode<R, N>::interRobotCommunicationCallback(
    const alliance_msgs::InterRobotCommunication& msg)
{
  InterRobotCommunicationEventConstPtr event(new InterRobotCommunicationEvent(
      boost::enable_shared_from_this<N>::shared_from_this(), msg));
  AllianceSubject<alliance_msgs::InterRobotCommunication>::notify(event);
}
}

#endif // _NODES_ALLIANCE_NODE_H_
