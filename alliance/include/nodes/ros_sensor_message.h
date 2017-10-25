#ifndef _ALLIANCE_NODE_ROS_SENSOR_MESSAGE_H_
#define _ALLIANCE_NODE_ROS_SENSOR_MESSAGE_H_

#include <sstream>
#include "alliance/sensor.h"
#include <alliance_msgs/SensoryFeedback.h>
#include <ros/node_handle.h>
#include <utilities/functions/unary_sample_holder.h>

namespace nodes
{
template <typename M> class ROSSensorMessage : public alliance::Sensor
{
public:
  typedef boost::shared_ptr<ROSSensorMessage<M> > Ptr;
  typedef boost::shared_ptr<ROSSensorMessage<M> const> ConstPtr;
  ROSSensorMessage();
  virtual ~ROSSensorMessage();
  virtual void readParameters();
  M getMsg() const;
  virtual bool isUpToDate() const;

protected:
  M msg_;

private:
  typedef utilities::functions::UnarySampleHolder SampleHolder;
  typedef utilities::functions::UnarySampleHolderPtr SampleHolderPtr;
  SampleHolderPtr up_to_date_;
  ros::NodeHandlePtr nh_;
  ros::Subscriber sensor_sub_;
  void sensorCallback(const M& msg);
};

template <typename M>
ROSSensorMessage<M>::ROSSensorMessage() {}

template <typename M> ROSSensorMessage<M>::~ROSSensorMessage()
{
  sensor_sub_.shutdown();
}

template <typename M> void ROSSensorMessage<M>::readParameters()
{
  ros::NodeHandle pnh("~/sensors");
  int size;
  pnh.param("size", size, 0);
  std::string topic_name;
  std::stringstream ss;
  for (int i(0); i < size; i++)
  {
    ss << "sensor" << i << "/";
    pnh.param(ss.str() + "topic_name", topic_name, std::string(""));
    topic_name = ns_ + "/" + topic_name;
    if (topic_name == id_)
    {
      nh_.reset(new ros::NodeHandle());
      break;
    }
    ss.str("");
  }
  if (!nh_)
  {
    throw utilities::Exception("There is no parameters of the " + id_ + " sensor.");
  }
  double buffer_horizon, timeout_duration;
  pnh.param("buffer_horizon", buffer_horizon, 5.0);
  if (buffer_horizon <= 0.0)
  {
    ROS_WARN(
        "The sensor buffer horizon must be positive.");
    buffer_horizon = 5.0;
  }
  pnh.param("timeout_duration", timeout_duration, 1.0);
  if (timeout_duration < 0.0)
  {
    ROS_WARN(
        "The sensor message timeout duration must be positive.");
    timeout_duration = 1.0;
  }
  up_to_date_.reset(
      new SampleHolder(topic_name, ros::Duration(timeout_duration), ros::Duration(buffer_horizon)));
  sensor_sub_ = nh_->subscribe(topic_name, 1,
                               &ROSSensorMessage<M>::sensorCallback, this);
}

template <typename M> M ROSSensorMessage<M>::getMsg() const { return msg_; }

template <typename M> bool ROSSensorMessage<M>::isUpToDate() const
{
  return up_to_date_->getValue();
}

template <typename M> void ROSSensorMessage<M>::sensorCallback(const M& msg)
{
  up_to_date_->update(ros::Time::now());
  msg_ = msg;
}
}

#endif // _ALLIANCE_NODE_ROS_SENSOR_MESSAGE_H_
