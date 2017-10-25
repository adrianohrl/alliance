#ifndef _ALLIANCE_SENSORY_EVALUATOR_H_
#define _ALLIANCE_SENSORY_EVALUATOR_H_

#include "alliance/sensor.h"
#include "alliance/task.h"
#include <alliance_msgs/SensoryFeedback.h>
#include <list>
#include <ros/publisher.h>

namespace alliance
{
class BehavedRobot;
typedef boost::shared_ptr<BehavedRobot> BehavedRobotPtr;
typedef boost::shared_ptr<BehavedRobot const> BehavedRobotConstPtr;

class SensoryEvaluator
{
public:
  SensoryEvaluator();
  virtual ~SensoryEvaluator();
  virtual void initialize(const ros::NodeHandlePtr& nh,
                          const BehavedRobotPtr& robot, const Task& task,
                          const std::list<std::string> &sensors);
  void process();
  virtual bool isApplicable() = 0;
  virtual SensorPtr getSensor(const std::string& sensor_id) const;

protected:
  typedef std::list<SensorPtr>::iterator iterator;
  typedef std::list<SensorPtr>::const_iterator const_iterator;
  std::list<SensorPtr> sensors_;

private:
  ros::NodeHandlePtr nh_;
  ros::Publisher sensory_feedback_pub_;
  alliance_msgs::SensoryFeedback sensory_feedback_msg_;
  bool contains(const std::string &sensor_id) const;
};

typedef boost::shared_ptr<SensoryEvaluator> SensoryEvaluatorPtr;
typedef boost::shared_ptr<SensoryEvaluator const> SensoryEvaluatorConstPtr;
}

#endif // _ALLIANCE_SENSORY_EVALUATOR_H_
