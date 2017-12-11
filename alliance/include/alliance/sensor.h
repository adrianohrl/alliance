#ifndef _ALLIANCE_SENSOR_H_
#define _ALLIANCE_SENSOR_H_

#include <ros/time.h>

namespace alliance
{
class Sensor
{
public:
  Sensor();
  virtual ~Sensor();
  virtual void initialize(const std::string& ns, const std::string& name,
                          const std::string& id);
  virtual void readParameters();
  std::string getNamespace() const;
  std::string getName() const;
  std::string getId() const;
  virtual bool isUpToDate() const;
  virtual bool operator==(const Sensor& sensor) const;
  virtual bool operator!=(const Sensor& sensor) const;

protected:
  std::string ns_;
  std::string name_;
  std::string id_;
};

typedef boost::shared_ptr<Sensor> SensorPtr;
typedef boost::shared_ptr<Sensor const> SensorConstPtr;
}

#endif // _ALLIANCE_SENSOR_H_
