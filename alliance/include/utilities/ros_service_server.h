/**
 *  This header file defines and implements the ROSServiceServer abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_ROS_SERVICE_SERVER_H_
#define _UTILITIES_ROS_SERVICE_SERVER_H_

#include <ros/ros.h>

namespace utilities
{
template <typename Request, typename Response> class ROSServiceServer
{
public:
  ROSServiceServer(const ros::ServiceServer server);
  virtual ~ROSServiceServer();
  std::string getName() const;
  std::string str() const;
  const char* c_str() const;

private:
  std::string name_;
  ros::ServiceServer server_;
  virtual bool callback(Request& request, Response& response) = 0;
};

template <typename Request, typename Response>
ROSServiceServer<Request, Response>::ROSServiceServer(
    const ros::ServiceServer server)
    : server_(server)
{
}

template <typename Request, typename Response>
ROSServiceServer<Request, Response>::~ROSServiceServer()
{
  server_.shutdown();
}

template <typename Request, typename Response>
std::string ROSServiceServer<Request, Response>::getName() const
{
  return name_;
}

template <typename Request, typename Response>
std::string ROSServiceServer<Request, Response>::str() const
{
  return name_;
}

template <typename Request, typename Response>
const char* ROSServiceServer<Request, Response>::c_str() const
{
  return str().c_str();
}
}

#endif // _UTILITIES_ROS_SERVICE_SERVER_H_
