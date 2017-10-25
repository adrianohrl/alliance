#include "alliance/layered_behaviour_set.h"
#include "alliance/behaved_robot.h"
#include <utilities/exception.h>

namespace alliance
{
LayeredBehaviourSet::LayeredBehaviourSet(const BehavedRobotPtr& robot,
                                         const TaskPtr& task,
                                         const ros::Duration& buffer_horizon,
                                         const ros::Duration& timeout_duration)
    : BehaviourSetInterface<BehavedRobot>::BehaviourSetInterface(
          robot, task, buffer_horizon, timeout_duration),
      AllianceObserver<alliance_msgs::InterRobotCommunication>::
          AllianceObserver(ns_),
      robot_(robot), task_(task), layer_loader_("alliance", "alliance::Layer"),
      sensory_evaluator_loader_("alliance", "alliance::SensoryEvaluator")
{
  for (Task::const_iterator it(task->begin()); it != task->end(); it++)
  {
    addLayer(*it);
  }
}

LayeredBehaviourSet::~LayeredBehaviourSet() {}

void LayeredBehaviourSet::preProcess()
{
  if (!sensory_evaluator_)
  {
    throw utilities::Exception("The sensory evaluator of " + str() +
                               " layered behaviour set has not been setted.");
  }
  sensory_evaluator_->process();
}

void LayeredBehaviourSet::process()
{
  for (layers_iterator it(layers_.begin()); it != layers_.end(); it++)
  {
    LayerPtr layer(*it);
    layer->process();
  }
}

void LayeredBehaviourSet::update(
    const nodes::InterRobotCommunicationEventConstPtr& event)
{
  if (!event->isRelated(*robot_) || !event->isRelated(*task_))
  {
    return;
  }
  setActive(true, event->getTimestamp());
}

void LayeredBehaviourSet::addLayer(const std::string& plugin_name)
{
  if (contains(plugin_name))
  {
    throw utilities::Exception("This layer already exists.");
  }
  try
  {
    LayerPtr layer(layer_loader_.createInstance(plugin_name.c_str()));
    layer->initialize(robot_->getId(), plugin_name);
    addLayer(layer);
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s",
              ex.what());
  }
}

void LayeredBehaviourSet::addLayer(const LayerPtr& layer)
{
  layers_.push_back(layer);
  ROS_DEBUG_STREAM("Loaded " << layer->getName() << " layer plugin to the "
                             << *this << " behaviour set.");
}

void LayeredBehaviourSet::setSensoryEvaluator(
    const ros::NodeHandlePtr& nh, const std::string& plugin_name,
    const std::list<std::string>& sensors)
{
  try
  {
    sensory_evaluator_ =
        sensory_evaluator_loader_.createInstance(plugin_name.c_str());
    sensory_evaluator_->initialize(nh, robot_, *task_, sensors);
    ROS_DEBUG_STREAM("Loaded " << plugin_name
                               << " sensory evaluator plugin to publish "
                               << *task_ << "'s sensory feedback.");
    for (layers_iterator it(layers_.begin()); it != layers_.end(); it++)
    {
      LayerPtr layer(*it);
      layer->setEvaluator(sensory_evaluator_);
    }
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    ROS_ERROR_STREAM("Could not load " << plugin_name << ". " << ex.what());
  }
}

bool LayeredBehaviourSet::contains(const std::string& plugin_name) const
{
  for (layers_const_iterator it(layers_.begin()); it != layers_.end(); it++)
  {
    LayerPtr layer(*it);
    if (layer->getName() == plugin_name)
    {
      return true;
    }
  }
  return false;
}
}
