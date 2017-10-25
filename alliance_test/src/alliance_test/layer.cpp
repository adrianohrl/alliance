#include "alliance_test/layer.h"

namespace alliance_test
{
Layer::Layer() : nh_(new ros::NodeHandle()) {}

Layer::~Layer() { velocity_pub_.shutdown(); }

void Layer::initialize(const std::string& ns, const std::string& name)
{
  ns_ = ns;
  alliance::Layer::initialize(ns_, name);
  velocity_pub_ = nh_->advertise<geometry_msgs::Twist>(ns + "/cmd_vel", 10);
}

void Layer::setEvaluator(const alliance::SensoryEvaluatorPtr& evaluator)
{
  alliance::Layer::setEvaluator(evaluator);
  odometry_ = boost::dynamic_pointer_cast<Odometry>(
      evaluator->getSensor(ns_ + "/odom"));
  sonars_ = boost::dynamic_pointer_cast<PointCloud>(
      evaluator->getSensor(ns_ + "/sonar"));
}

void Layer::process() { velocity_pub_.publish(velocity_msg_); }

void Layer::setVelocity(double vx, double wz)
{
  velocity_msg_.linear.x = vx;
  velocity_msg_.angular.z = wz;
}
}
