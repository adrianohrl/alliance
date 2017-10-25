#ifndef _ALLIANCE_LAYERED_BEHAVIOUR_SET_H_
#define _ALLIANCE_LAYERED_BEHAVIOUR_SET_H_

#include "alliance/behaviour_set_interface.h"
#include "alliance/layer.h"
#include "alliance/sensory_evaluator.h"
#include <list>
#include "nodes/alliance_observer.h"
#include <pluginlib/class_loader.h>

namespace alliance
{
class BehavedRobot;
typedef boost::shared_ptr<BehavedRobot> BehavedRobotPtr;
typedef boost::shared_ptr<BehavedRobot const> BehavedRobotConstPtr;

class LayeredBehaviourSet : public BehaviourSetInterface<BehavedRobot>,
                            public nodes::InterRobotCommunicationObserver
{
public:
  LayeredBehaviourSet(const BehavedRobotPtr& robot, const TaskPtr& task,
                      const ros::Duration& buffer_horizon,
                      const ros::Duration& timeout_duration);
  virtual ~LayeredBehaviourSet();
  virtual void preProcess();
  virtual void process();
  virtual void update(const nodes::InterRobotCommunicationEventConstPtr& event);
  void addLayer(const std::string& plugin_name);
  void addLayer(const LayerPtr& layer);
  void setSensoryEvaluator(const ros::NodeHandlePtr& nh,
                           const std::string& plugin_name,
                           const std::list<std::string> &sensors);

private:
  typedef std::list<LayerPtr>::iterator layers_iterator;
  typedef std::list<LayerPtr>::const_iterator layers_const_iterator;
  BehavedRobotPtr robot_;
  TaskPtr task_;
  pluginlib::ClassLoader<Layer> layer_loader_;
  std::list<LayerPtr> layers_;
  pluginlib::ClassLoader<SensoryEvaluator> sensory_evaluator_loader_;
  SensoryEvaluatorPtr sensory_evaluator_;
  bool contains(const std::string& plugin_name) const;
};

typedef boost::shared_ptr<LayeredBehaviourSet> LayeredBehaviourSetPtr;
typedef boost::shared_ptr<LayeredBehaviourSet const>
    LayeredBehaviourSetConstPtr;
}

#endif // _ALLIANCE_LAYERED_BEHAVIOUR_SET_H_
