/**
 *  This header file defines the Simulation abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_SIMULATION_H_
#define _UTILITIES_SIMULATION_H_

#include <ros/time.h>

namespace utilities
{
class Simulation
{
public:
  virtual ~Simulation();
  virtual void update(const ros::Time& timestamp) = 0;
  ros::Duration getSimulationElapsedDuration(
      const ros::Time& timestamp = ros::Time::now()) const;
  ros::Time getSimulationStartTimestamp() const;
  virtual std::string str() const = 0;
  const char* c_str() const;
  friend std::ostream& operator<<(std::ostream& out,
                                  const Simulation& simulation);

protected:
  ros::Time start_timestamp_;
  ros::Time last_update_timestamp_;
  Simulation();
  Simulation(const Simulation& simulation);
};
}

#endif // _UTILITIES_SIMULATION_H_
