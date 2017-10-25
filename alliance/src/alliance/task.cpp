#include "alliance/task.h"

namespace alliance
{
Task::Task(const std::string& id, const std::string& name)
    : HasName::HasName(name, id)
{
}

Task::Task(const Task& task)
    : HasName::HasName(task), needed_layers_(task.needed_layers_)
{
}

Task::~Task() {}

void Task::addNeededLayer(const std::string& layer_name)
{
  if (!contains(layer_name))
  {
    needed_layers_.push_back(layer_name);
  }
}

std::size_t Task::size() const { return needed_layers_.size(); }

bool Task::empty() const { return needed_layers_.empty(); }

Task::iterator Task::begin() { return needed_layers_.begin(); }

Task::const_iterator Task::begin() const { return needed_layers_.begin(); }

Task::iterator Task::end() { return needed_layers_.end(); }

Task::const_iterator Task::end() const { return needed_layers_.end(); }

bool Task::contains(const std::string& layer_name) const
{
  for (const_iterator it(needed_layers_.begin()); it != needed_layers_.end();
       it++)
  {
    if (*it == layer_name)
    {
      return true;
    }
  }
  return false;
}
}
