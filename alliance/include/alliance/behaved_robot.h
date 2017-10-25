#ifndef _ALLIANCE_BEHAVED_ROBOT_H_
#define _ALLIANCE_BEHAVED_ROBOT_H_

#include "alliance/layered_behaviour_set.h"
#include "alliance/robot_interface.h"
#include <alliance_msgs/InterRobotCommunication.h>
#include "nodes/ros_sensor_message.h"

namespace alliance
{
class BehavedRobot : public RobotInterface<LayeredBehaviourSet>
{
public:
  typedef std::list<SensorPtr>::iterator sensors_iterator;
  typedef std::list<SensorPtr>::const_iterator sensors_const_iterator;
  BehavedRobot(const std::string& id, const std::string& name,
               const std::string& ns);
  virtual ~BehavedRobot();
  SensorPtr getSensor(const std::string& sensor_id) const;
  void addSensor(const std::string& plugin_name, const std::string &topic_name);
  void addSensor(const SensorPtr& sensor);

private:
  std::list<SensorPtr> sensors_;
  pluginlib::ClassLoader<Sensor> loader_;
  bool contains(const std::string& sensor_id) const;
};

typedef boost::shared_ptr<BehavedRobot> BehavedRobotPtr;
typedef boost::shared_ptr<BehavedRobot const> BehavedRobotConstPtr;
}

#endif // _ALLIANCE_BEHAVED_ROBOT_H_
