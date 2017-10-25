#ifndef _ALLIANCE_TEST_LAYER_H_
#define _ALLIANCE_TEST_LAYER_H_

#include <alliance/layer.h>
#include <geometry_msgs/Twist.h>
#include "alliance_test/odometry.h"
#include "alliance_test/point_cloud.h"

namespace alliance_test
{
class Layer : public alliance::Layer
{
public:
  Layer();
  virtual ~Layer();
  virtual void initialize(const std::string& ns, const std::string& name);
  virtual void setEvaluator(const alliance::SensoryEvaluatorPtr& evaluator);
  virtual void process();

protected:
  std::string ns_;
  ros::NodeHandlePtr nh_;
  OdometryPtr odometry_;
  PointCloudPtr sonars_;
  ros::Publisher velocity_pub_;
  void setVelocity(double vx = 0.0, double wz = 0.0);

private:
  geometry_msgs::Twist velocity_msg_;
};
}

#endif // _ALLIANCE_TEST_LAYER_H_
