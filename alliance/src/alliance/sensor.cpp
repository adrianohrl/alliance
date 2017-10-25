#include "alliance/sensor.h"

namespace alliance
{
Sensor::Sensor() {}

Sensor::~Sensor() {}

void Sensor::initialize(const std::string& ns, const std::string& name,
                        const std::string& id)
{
  ns_ = ns;
  name_ = name;
  id_ = ns + "/" + id;
  readParameters();
}

void Sensor::readParameters() {}

std::string Sensor::getNamespace() const { return ns_; }

std::string Sensor::getName() const { return name_; }

std::string Sensor::getId() const { return id_; }

bool Sensor::isUpToDate() const { return true; }

bool Sensor::operator==(const Sensor& sensor) const
{
  return id_ == sensor.id_;
}

bool Sensor::operator!=(const Sensor& sensor) const
{
  return !(*this == sensor);
}
}
