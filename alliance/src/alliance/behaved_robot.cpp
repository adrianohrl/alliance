#include "alliance/behaved_robot.h"

namespace alliance
{
BehavedRobot::BehavedRobot(const std::string& id, const std::string& name,
                           const std::string& ns)
    : RobotInterface<LayeredBehaviourSet>::RobotInterface(id, name, ns),
      loader_("alliance", "alliance::Sensor")
{
}

BehavedRobot::~BehavedRobot() {}

SensorPtr BehavedRobot::getSensor(const std::string &sensor_id) const
{
  for (sensors_const_iterator it(sensors_.begin()); it != sensors_.end(); it++)
  {
    SensorPtr sensor(*it);
    if (sensor->getId() == sensor_id)
    {
      return sensor;
    }
  }
  return SensorPtr();
}

void BehavedRobot::addSensor(const std::string& plugin_name,
                             const std::string& topic_name)
{
  if (contains(id_ + "/" + topic_name))
  {
    throw utilities::Exception("This sensor already exists.");
  }
  try
  {
    SensorPtr sensor(loader_.createInstance(plugin_name.c_str()));
    sensor->initialize(id_, plugin_name, topic_name);
    addSensor(sensor);
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s",
              ex.what());
  }
}

void BehavedRobot::addSensor(const SensorPtr& sensor)
{
  sensors_.push_back(sensor);
  ROS_DEBUG_STREAM("Loaded " << sensor->getName() << " sensor plugin to the "
                             << *this << " behaved robot.");
}

bool BehavedRobot::contains(const std::string& sensor_id) const
{
  for (sensors_const_iterator it(sensors_.begin()); it != sensors_.end(); it++)
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
