#include "nodes/low_level_node.h"

namespace nodes
{

LowLevelNode::LowLevelNode(const ros::NodeHandlePtr& nh, const ros::Rate& rate)
    : AllianceNode<alliance::BehavedRobot, LowLevelNode>::AllianceNode(nh, rate)
{
}

LowLevelNode::~LowLevelNode() {}

void LowLevelNode::readParameters()
{
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
    ss << "layers/";
    int layers_size;
    pnh.param(ss.str() + "size", layers_size, 0);
    std::string plugin_name;
    for (int j(0); j < layers_size; j++)
    {
      std::stringstream sss;
      sss << ss.str() << "layer" << j << "/";
      pnh.param(sss.str() + "plugin_name", plugin_name, std::string(""));
      task->addNeededLayer(plugin_name);
    }
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
  robot_.reset(new alliance::BehavedRobot(ns, name, ns));
  double buffer_horizon, timeout_duration;
  pnh.param("buffer_horizon", buffer_horizon, 5.0);
  if (buffer_horizon <= 0.0)
  {
    ROS_WARN(
        "The active buffer horizon of the behaviour set must be positive.");
    buffer_horizon = 5.0;
  }
  pnh.param("timeout_duration", timeout_duration, 2.0);
  if (timeout_duration < 0.0)
  {
    ROS_WARN("The active timeout duration of the behaviour set must not be "
             "negative.");
    timeout_duration = 2.0;
  }
  pnh = ros::NodeHandle("~/sensors");
  pnh.param("size", size, 0);
  for (int i(0); i < size; i++)
  {
    std::stringstream ss;
    ss << "sensor" << i << "/";
    std::string plugin_name, topic_name;
    pnh.param(ss.str() + "plugin_name", plugin_name, std::string(""));
    if (plugin_name.empty())
    {
      ROS_ERROR("The sensor plugin name parameter must not be empty.");
      continue;
    }
    pnh.param(ss.str() + "topic_name", topic_name, std::string(""));
    if (plugin_name.empty())
    {
      ROS_ERROR("The sensor topic name parameter must not be empty.");
      continue;
    }
    robot_->addSensor(plugin_name, topic_name);
  }
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
    ss << "motivational_behaviour/sensory_feedback/";
    std::string plugin_name;
    pnh.param(ss.str() + "plugin_name", plugin_name, std::string(""));
    if (plugin_name.empty())
    {
      ROS_ERROR(
          "The sensor evaluator plugin name parameter must not be empty.");
      continue;
    }
    ss << "sensors/";
    int sensors_size;
    pnh.param(ss.str() + "size", sensors_size, 0);
    std::list<std::string> sensors;
    std::string topic_name;
    for (int j(0); j < sensors_size; j++)
    {
      std::stringstream sss;
      sss << ss.str() << "sensor" << j << "/";
      pnh.param(sss.str() + "topic_name", topic_name, std::string(""));
      if (topic_name.empty())
      {
        ROS_ERROR(
            "The sensor evaluator plugin name parameter must not be empty.");
        continue;
      }
      sensors.push_back(topic_name);
    }
    std::list<alliance::TaskPtr>::const_iterator it(tasks.begin());
    alliance::LayeredBehaviourSetPtr behaviour_set;
    while (it != tasks.end())
    {
      alliance::TaskPtr task(*it);
      if (task->getId() == id)
      {
        behaviour_set.reset(new alliance::LayeredBehaviourSet(
            robot_, task, ros::Duration(buffer_horizon),
            ros::Duration(timeout_duration)));
        behaviour_set->setSensoryEvaluator(nh_, plugin_name, sensors);
        break;
      }
      it++;
    }
    robot_->addBehaviourSet(behaviour_set);
  }
  if (robot_->empty())
  {
    ROSNode::shutdown("None behaviour set was imported to " + robot_->str() +
                      " robot.");
  }
}

void LowLevelNode::init()
{
  AllianceNode<alliance::BehavedRobot, LowLevelNode>::init();
  /** registering beacon signal message observers **/
  for (alliance::BehavedRobot::iterator it(robot_->begin());
       it != robot_->end(); it++)
  {
    AllianceSubject<alliance_msgs::InterRobotCommunication>::registerObserver(
        *it);
  }
}
}
