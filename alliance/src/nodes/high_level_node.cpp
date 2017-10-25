#include "nodes/high_level_node.h"

namespace nodes
{
HighLevelNode::HighLevelNode(const ros::NodeHandlePtr& nh,
                             const ros::Rate& rate)
    : AllianceNode<alliance::Robot, HighLevelNode>::AllianceNode(nh, rate),
      AllianceSubject<alliance_msgs::SensoryFeedback>::AllianceSubject(
          ros::this_node::getName()),
      broadcasting_(false)
{
}

HighLevelNode::~HighLevelNode()
{
  broadcast_timer_.stop();
  inter_robot_communication_pub_.shutdown();
  motivation_pub_.shutdown();
  sensory_feedback_sub_.shutdown();
}

void HighLevelNode::readParameters()
{
  ROSNode::readParameters();
  ros::NodeHandle pnh("~/tasks");
  int size;
  pnh.param("size", size, 0);
  std::string id, name;
  std::list<alliance::TaskPtr> tasks;
  for (int i(0); i < size; i++)
  {
    std::stringstream ss;
    ss << "task" << i << "/";
    pnh.param(ss.str() + "id", id, std::string(""));
    if (id.empty())
    {
      ROS_ERROR("The task's id must not be empty.");
      continue;
    }
    pnh.param(ss.str() + "name", name, std::string(""));
    alliance::TaskPtr task(new alliance::Task(id, name.empty() ? id : name));
    tasks.push_back(task);
  }
  if (tasks.empty())
  {
    ROSNode::shutdown("Not found any task info as a ROS parameter.");
    return;
  }
  pnh = ros::NodeHandle("~");
  pnh.param("id", id, std::string(""));
  pnh.param("name", name, std::string(""));
  if (id.empty())
  {
    ROSNode::shutdown("Not found robot id as a ROS parameter.");
    return;
  }
  std::string ns(ros::this_node::getName()), aux(id + "/alliance");
  ns.replace(ns.find_last_of('/'), ns.npos, "");
  if (!std::equal(aux.rbegin(), aux.rend(), ns.rbegin()))
  {
    ROSNode::shutdown("Invalid ROS namespace. It must end with '" + id + "'.");
    return;
  }
  aux = "/alliance";
  ns = ns.substr(0, ns.size() - aux.size());
  robot_.reset(new alliance::Robot(ns, name, ns));
  double broadcast_rate;
  pnh.param("broadcast_rate", broadcast_rate, 0.0);
  if (broadcast_rate <= 0.0)
  {
    ROSNode::shutdown(
        "The robot's inter communication broadcast rate must be positive.");
    return;
  }
  robot_->setBroadcastRate(ros::Rate(broadcast_rate));
  double buffer_horizon, timeout_duration;
  pnh.param("buffer_horizon", buffer_horizon, 5.0);
  if (buffer_horizon <= 0.0)
  {
    ROS_WARN(
        "The active buffer horizon of the behaviour set must be positive.");
    buffer_horizon = 5.0;
  }
  pnh.param("timeout_duration", timeout_duration, 0.0);
  if (timeout_duration < 0.0)
  {
    ROS_WARN(
        "The robot's inter communication timeout duration must be positive.");
    timeout_duration = 0.0;
  }
  robot_->setTimeoutDuration(ros::Duration(timeout_duration));
  pnh = ros::NodeHandle("~/behaviour_sets");
  pnh.param("size", size, 0);
  for (int i(0); i < size; i++)
  {
    std::stringstream ss;
    ss << "behaviour_set" << i << "/";
    pnh.param(ss.str() + "task_id", id, std::string(""));
    if (id.empty())
    {
      ROS_ERROR("The behaviour set's task id must not be empty.");
      continue;
    }
    std::list<alliance::TaskPtr>::const_iterator it(tasks.begin());
    alliance::BehaviourSetPtr behaviour_set;
    while (it != tasks.end())
    {
      alliance::TaskPtr task(*it);
      if (task->getId() == id)
      {
        behaviour_set.reset(new alliance::BehaviourSet(
            robot_, task, ros::Duration(buffer_horizon)));
        behaviour_set->init();
        break;
      }
      it++;
    }
    if (!behaviour_set)
    {
      ROS_ERROR("The behaviour set's task id is unknown.");
      continue;
    }
    double expected_duration;
    pnh.param(ss.str() + "task_expected_duration", expected_duration, 0.0);
    if (expected_duration <= 0.0)
    {
      ROS_ERROR("The behaviour set's task expected duration must be positive.");
      continue;
    }
    behaviour_set->setTaskExpectedDuration(ros::Duration(expected_duration));
    ss << "motivational_behaviour/";
    double threshold;
    pnh.param(ss.str() + "threshold", threshold, 0.0);
    if (threshold <= 0.0)
    {
      ROS_ERROR("The behaviour set's motivational behaviour's threshold must "
                "be positive.");
      continue;
    }
    behaviour_set->setActivationThreshold(threshold);
    double yielding_delay, giving_up_delay;
    pnh.param(ss.str() + "acquiescence/yielding_delay", yielding_delay, 0.0);
    if (yielding_delay < 0.0)
    {
      ROS_WARN("The robot's yielding delay must not be negative.");
      yielding_delay = 0.0;
    }
    pnh.param(ss.str() + "acquiescence/giving_up_delay", giving_up_delay, 0.0);
    if (giving_up_delay < 0.0)
    {
      ROS_WARN("The robot's yielding delay must not be negative.");
      giving_up_delay = 0.0;
    }
    behaviour_set->setAcquiescence(ros::Duration(yielding_delay),
                                   ros::Duration(giving_up_delay));
    double fast_rate;
    pnh.param(ss.str() + "impatience/fast_rate", fast_rate, 0.0);
    if (fast_rate <= 0.0)
    {
      ROS_ERROR("The robot's impatience fast rate must be positive.");
      continue;
    }
    behaviour_set->setImpatience(fast_rate);
    robot_->addBehaviourSet(behaviour_set);
  }
  if (robot_->empty())
  {
    ROSNode::shutdown("None behaviour set was imported to " + robot_->str() +
                      " robot.");
  }
}

void HighLevelNode::init()
{
  AllianceNode<alliance::Robot, HighLevelNode>::init();
  /** registering beacon signal message observers **/
  for (alliance::Robot::iterator it(robot_->begin()); it != robot_->end(); it++)
  {
    alliance::BehaviourSetPtr behaviour_set(*it);
    alliance::MotivationalBehaviourPtr motivational_behaviour(
        behaviour_set->getMotivationalBehaviour());
    AllianceSubject<alliance_msgs::InterRobotCommunication>::registerObserver(
        motivational_behaviour->getInterRobotCommunication());
    AllianceSubject<alliance_msgs::InterRobotCommunication>::registerObserver(
        motivational_behaviour->getImpatience());
    AllianceSubject<alliance_msgs::SensoryFeedback>::registerObserver(
        motivational_behaviour->getSensoryFeedback());
  }
  /** creating robot broadcast timer **/
  broadcast_timer_ = nh_->createTimer(
      robot_->getBroadcastRate().expectedCycleTime(),
      &HighLevelNode::broadcastTimerCallback, this, false, false);
  inter_robot_communication_pub_ =
      nh_->advertise<alliance_msgs::InterRobotCommunication>(
          "/alliance/inter_robot_communication", robot_->size());
  motivation_pub_ =
      nh_->advertise<alliance_msgs::Motivation>("/alliance/motivation", 1);
  sensory_feedback_sub_ =
      nh_->subscribe(robot_->getNamespace() + "/alliance/sensory_feedback", 10,
                     &HighLevelNode::sensoryFeedbackCallback, this);
}

void HighLevelNode::controlLoop()
{
  AllianceNode<alliance::Robot, HighLevelNode>::controlLoop();
  if (!robot_->isIdle() && !broadcasting_)
  {
    broadcast_timer_.start();
    broadcasting_ = true;
    ROS_INFO_STREAM("Started " << *robot_ << " broadcast timer for "
                                << *robot_->getExecutingTask() << ".");
  }
  for (alliance::Robot::iterator it(robot_->begin()); it != robot_->end(); it++)
  {
    alliance::BehaviourSetPtr behaviour_set(*it);
    motivation_pub_.publish(behaviour_set->getMotivationalBehaviour()->toMsg());
  }
}

void HighLevelNode::broadcastTimerCallback(const ros::TimerEvent& event)
{
  if (!robot_->getExecutingTask())
  {
    broadcast_timer_.stop();
    broadcasting_ = false;
    ROS_INFO_STREAM("Stopped " << *robot_ << " broadcast timer.");
    return;
  }
  alliance_msgs::InterRobotCommunication msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = robot_->getId();
  msg.task_id = robot_->getExecutingTask()->getId();
  msg.expected_duration = robot_->getExecutingTaskExpectedDuration();
  inter_robot_communication_pub_.publish(msg);
}

void HighLevelNode::sensoryFeedbackCallback(
    const alliance_msgs::SensoryFeedback& msg)
{
  SensoryFeedbackEventConstPtr event(
      new SensoryFeedbackEvent(shared_from_this(), msg));
  AllianceSubject<alliance_msgs::SensoryFeedback>::notify(event);
}
}
