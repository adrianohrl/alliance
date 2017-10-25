#include "alliance/sensory_evaluator.h"
#include "alliance/behaved_robot.h"

namespace alliance
{
SensoryEvaluator::SensoryEvaluator() {}

SensoryEvaluator::~SensoryEvaluator() { sensory_feedback_pub_.shutdown(); }

void SensoryEvaluator::initialize(const ros::NodeHandlePtr& nh,
                                  const BehavedRobotPtr& robot,
                                  const Task& task,
                                  const std::list<std::string>& sensors)
{
  nh_ = nh;
  sensory_feedback_pub_ = nh_->advertise<alliance_msgs::SensoryFeedback>(
      robot->getNamespace() + "/alliance/sensory_feedback", 1);
  sensory_feedback_msg_.header.frame_id = robot->getId();
  sensory_feedback_msg_.task_id = task.getId();
  for (std::list<std::string>::const_iterator it(sensors.begin());
       it != sensors.end(); it++)
  {
    if (!contains(*it))
    {
      SensorPtr sensor(robot->getSensor(robot->getId() + "/" + *it));
      if (!sensor)
      {
        ROS_ERROR_STREAM(*robot << " behaved robot does not have registered "
                                << *it << " sensor.");
        continue;
      }
      sensors_.push_back(sensor);
    }
  }
}

void SensoryEvaluator::process()
{
  sensory_feedback_msg_.header.stamp = ros::Time::now();
  sensory_feedback_msg_.applicable = isApplicable();
  sensory_feedback_pub_.publish(sensory_feedback_msg_);
}

SensorPtr SensoryEvaluator::getSensor(const std::string &sensor_id) const
{
  for (const_iterator it(sensors_.begin()); it != sensors_.end(); it++)
  {
    SensorPtr sensor(*it);
    if (sensor->getId() == sensor_id)
    {
      return sensor;
    }
  }
  return SensorPtr();
}

bool SensoryEvaluator::contains(const std::string& sensor_id) const
{
  for (const_iterator it(sensors_.begin()); it != sensors_.end(); it++)
  {
    SensorPtr sensor(*it);
    if (sensor->getId() == sensor_id)
    {
      return true;
    }
  }
  return false;
}
}
