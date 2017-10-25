#ifndef _ALLIANCE_TASK_H_
#define _ALLIANCE_TASK_H_

#include <boost/shared_ptr.hpp>
#include <list>
#include <utilities/has_name.h>

namespace alliance
{
class Task : public utilities::HasName
{
public:
  typedef std::list<std::string>::iterator iterator;
  typedef std::list<std::string>::const_iterator const_iterator;
  Task(const std::string& id, const std::string& name);
  Task(const Task& task);
  virtual ~Task();
  void addNeededLayer(const std::string& layer_name);
  std::size_t size() const;
  bool empty() const;
  iterator begin();
  const_iterator begin() const;
  iterator end();
  const_iterator end() const;

private:
  std::list<std::string> needed_layers_;
  bool contains(const std::string& layer_name) const;
};

typedef boost::shared_ptr<Task> TaskPtr;
typedef boost::shared_ptr<Task const> TaskConstPtr;
}

#endif // _ALLIANCE_TASK_H_
