/**
 *  This source file tests the main alliance namespace classes.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "alliance/alliance.h"
#include "nodes/alliance_subject.h"
#include "utilities/utilities.h"
#include <gtest/gtest.h>

using namespace alliance;

double tolerance(1e-4);
TaskPtr t1, t2;
RobotPtr r1, r2, r3;
MotivationalBehaviourPtr r1bs1_mb, r1bs2_mb, r2bs1_mb, r2bs2_mb, r3bs1_mb,
    r3bs2_mb;
nodes::InterRobotCommunicationSubjectPtr r1_subject, r2_subject, r3_subject;

void notify(const RobotPtr& robot, const TaskPtr& task,
            const ros::Time& timestamp)
{
  alliance_msgs::InterRobotCommunication msg;
  msg.header.frame_id = robot->getId();
  msg.task_id = task->getId();
  msg.header.stamp = timestamp;
  nodes::InterRobotCommunicationEventPtr event;
  event.reset(new nodes::InterRobotCommunicationEvent(r1_subject, msg));
  r1_subject->notify(event);
  event.reset(new nodes::InterRobotCommunicationEvent(r2_subject, msg));
  r2_subject->notify(event);
  event.reset(new nodes::InterRobotCommunicationEvent(r3_subject, msg));
  r3_subject->notify(event);
}

TEST(MotivationalBehaviour, impatience_reset)
{
  ImpatienceResetPtr r1bs1_reset(r1bs1_mb->getImpatienceReset());
  ImpatienceResetPtr r1bs2_reset(r1bs2_mb->getImpatienceReset());
  ImpatienceResetPtr r2bs1_reset(r2bs1_mb->getImpatienceReset());
  ImpatienceResetPtr r2bs2_reset(r2bs2_mb->getImpatienceReset());
  ImpatienceResetPtr r3bs1_reset(r3bs1_mb->getImpatienceReset());
  ImpatienceResetPtr r3bs2_reset(r3bs2_mb->getImpatienceReset());
  ros::Time timestamp(ros::Time::now());
  ROS_WARN_STREAM("timestamp: " << timestamp);
  /*EXPECT_FALSE(r1bs1_reset->isResetted(timestamp));
  EXPECT_FALSE(r1bs2_reset->isResetted(timestamp));
  EXPECT_FALSE(r2bs1_reset->isResetted(timestamp));
  EXPECT_FALSE(r2bs2_reset->isResetted(timestamp));
  EXPECT_FALSE(r3bs1_reset->isResetted(timestamp));
  EXPECT_FALSE(r3bs2_reset->isResetted(timestamp));*/
  notify(r1, t1, timestamp + ros::Duration(0.5));
  ROS_WARN_STREAM("timestamp + ros::Duration(0.5): " << timestamp +
                                                            ros::Duration(0.5));
  ROS_WARN_STREAM(
      "timestamp + ros::Duration(0.51): " << timestamp + ros::Duration(0.51));
  /*EXPECT_FALSE(r1bs1_reset->isResetted(timestamp + ros::Duration(0.51)));
  EXPECT_FALSE(r1bs2_reset->isResetted(timestamp + ros::Duration(0.51)));*/
  EXPECT_TRUE(r2bs1_reset->isResetted(timestamp + ros::Duration(0.55)));
  // EXPECT_FALSE(r2bs2_reset->isResetted(timestamp + ros::Duration(0.51)));
  EXPECT_TRUE(r3bs1_reset->isResetted(timestamp + ros::Duration(0.55)));
  // EXPECT_FALSE(r3bs2_reset->isResetted(timestamp + ros::Duration(0.51)));
  /*EXPECT_FALSE(r2bs1_reset->isResetted(timestamp + ros::Duration(0.75)));
  EXPECT_FALSE(r3bs1_reset->isResetted(timestamp + ros::Duration(0.75)));
  /*notify(r1, t1, timestamp + ros::Duration(1.0));
  notify(r2, t2, timestamp + ros::Duration(1.0));
  EXPECT_FALSE(r2bs1_reset->isResetted(timestamp + ros::Duration(1.0)));
  EXPECT_FALSE(r3bs1_reset->isResetted(timestamp + ros::Duration(1.0)));
  EXPECT_TRUE(r1bs2_reset->isResetted(timestamp + ros::Duration(1.0)));
  EXPECT_TRUE(r3bs2_reset->isResetted(timestamp + ros::Duration(1.0)));
  EXPECT_FALSE(r1bs2_reset->isResetted(timestamp + ros::Duration(1.25)));
  EXPECT_FALSE(r3bs2_reset->isResetted(timestamp + ros::Duration(1.25)));
  notify(r2, t2, timestamp + ros::Duration(1.5));
  EXPECT_FALSE(r1bs2_reset->isResetted(timestamp + ros::Duration(1.5)));
  EXPECT_FALSE(r3bs2_reset->isResetted(timestamp + ros::Duration(1.5)));*/
}

TEST(MotivationalBehaviour, activity_suppression)
{
  ActivitySuppressionPtr r1bs1_suppression(r1bs1_mb->getActivitySuppression());
  ActivitySuppressionPtr r1bs2_suppression(r1bs2_mb->getActivitySuppression());
  ActivitySuppressionPtr r2bs1_suppression(r2bs1_mb->getActivitySuppression());
  ActivitySuppressionPtr r2bs2_suppression(r2bs2_mb->getActivitySuppression());
  ActivitySuppressionPtr r3bs1_suppression(r3bs1_mb->getActivitySuppression());
  ActivitySuppressionPtr r3bs2_suppression(r3bs2_mb->getActivitySuppression());
  ros::Time timestamp(ros::Time::now());
  EXPECT_FALSE(r1bs1_suppression->isSuppressed(timestamp));
  EXPECT_FALSE(r1bs2_suppression->isSuppressed(timestamp));
  EXPECT_FALSE(r2bs1_suppression->isSuppressed(timestamp));
  EXPECT_FALSE(r2bs2_suppression->isSuppressed(timestamp));
  EXPECT_FALSE(r3bs1_suppression->isSuppressed(timestamp));
  EXPECT_FALSE(r3bs2_suppression->isSuppressed(timestamp));
  notify(r1, t1, timestamp + ros::Duration(0.5));
  EXPECT_FALSE(r1bs1_suppression->isSuppressed(timestamp + ros::Duration(0.5)));
  EXPECT_TRUE(r1bs2_suppression->isSuppressed(timestamp + ros::Duration(0.5)));
  EXPECT_FALSE(r2bs1_suppression->isSuppressed(timestamp + ros::Duration(0.5)));
  EXPECT_FALSE(r2bs2_suppression->isSuppressed(timestamp + ros::Duration(0.5)));
  notify(r1, t1, timestamp + ros::Duration(1.0));
  EXPECT_FALSE(r1bs1_suppression->isSuppressed(timestamp + ros::Duration(1.0)));
  EXPECT_TRUE(r1bs2_suppression->isSuppressed(timestamp + ros::Duration(1.0)));
  EXPECT_TRUE(r2bs1_suppression->isSuppressed(timestamp + ros::Duration(1.0)));
  EXPECT_FALSE(r2bs2_suppression->isSuppressed(timestamp + ros::Duration(1.0)));
}

TEST(MotivationalBehaviour, impatience)
{
  ImpatiencePtr r1bs1_impatience(r1bs1_mb->getImpatience());
  ImpatiencePtr r1bs2_impatience(r1bs2_mb->getImpatience());
  ImpatiencePtr r2bs1_impatience(r2bs1_mb->getImpatience());
  ImpatiencePtr r2bs2_impatience(r2bs2_mb->getImpatience());
  ImpatiencePtr r3bs1_impatience(r3bs1_mb->getImpatience());
  ImpatiencePtr r3bs2_impatience(r3bs2_mb->getImpatience());
  ros::Time t0(ros::Time::now());
  int counter(0);
  for (ros::Time t(t0);
       t - t0 < r2->getTimeoutDuration() &&
       t - t0 < r2bs1_impatience->getReliabilityDuration(r1->getId(), t) &&
       t - t0 < r3->getTimeoutDuration() &&
       t - t0 < r3bs1_impatience->getReliabilityDuration(r1->getId(), t);
       t += r1->getBroadcastRate().expectedCycleTime())
  {
    notify(r1, t1, t);
    EXPECT_GE(tolerance, fabs(counter * r1bs1_impatience->getFastRate(t) -
                              r1bs1_impatience->getLevel(t)));
    EXPECT_GE(tolerance,
              fabs(counter * r2bs1_impatience->getSlowRate(r1->getId()) -
                   r2bs1_impatience->getLevel(t)));
    EXPECT_GE(tolerance,
              fabs(counter * r3bs1_impatience->getSlowRate(r1->getId()) -
                   r3bs1_impatience->getLevel(t)));
    counter++;
  }
  counter = 0;
  for (ros::Time t(t0);
       t - t0 < r1->getTimeoutDuration() &&
       t - t0 < r1bs2_impatience->getReliabilityDuration(r2->getId(), t) &&
       t - t0 < r3->getTimeoutDuration() &&
       t - t0 < r3bs2_impatience->getReliabilityDuration(r2->getId(), t);
       t += r2->getBroadcastRate().expectedCycleTime())
  {
    notify(r2, t2, t);
    EXPECT_GE(tolerance, fabs(counter * r2bs2_impatience->getFastRate(t) -
                              r2bs2_impatience->getLevel(t)));
    EXPECT_GE(tolerance,
              fabs(counter * r1bs2_impatience->getSlowRate(r2->getId()) -
                   r1bs2_impatience->getLevel(t)));
    EXPECT_GE(tolerance,
              fabs(counter * r3bs2_impatience->getSlowRate(r2->getId()) -
                   r3bs2_impatience->getLevel(t)));
    counter++;
  }
}

TEST(MotivationalBehaviour, inter_communication_monitor)
{
  InterRobotCommunicationPtr r1bs1_monitor(
      r1bs1_mb->getInterRobotCommunication());
  InterRobotCommunicationPtr r1bs2_monitor(
      r1bs2_mb->getInterRobotCommunication());
  InterRobotCommunicationPtr r2bs1_monitor(
      r2bs1_mb->getInterRobotCommunication());
  InterRobotCommunicationPtr r2bs2_monitor(
      r2bs2_mb->getInterRobotCommunication());
  InterRobotCommunicationPtr r3bs1_monitor(
      r3bs1_mb->getInterRobotCommunication());
  InterRobotCommunicationPtr r3bs2_monitor(
      r3bs2_mb->getInterRobotCommunication());
  ros::Time timestamp(ros::Time::now() + ros::Duration(200.0));
  notify(r1, t1, timestamp + ros::Duration(0.5));
  notify(r2, t1, timestamp + ros::Duration(0.5));
  notify(r3, t2, timestamp + ros::Duration(0.5));
  notify(r2, t1, timestamp + ros::Duration(1.0));
  notify(r3, t2, timestamp + ros::Duration(1.0));
  notify(r3, t2, timestamp + ros::Duration(1.5));
  notify(r3, t2, timestamp + ros::Duration(2.0));
  notify(r3, t2, timestamp + ros::Duration(2.5));
  notify(r3, t2, timestamp + ros::Duration(3.0));
  notify(r3, t2, timestamp + ros::Duration(3.5));
  EXPECT_TRUE(r1bs1_monitor->received(r2->getId(), timestamp,
                                      timestamp + ros::Duration(3.5)));
  EXPECT_TRUE(r1bs2_monitor->received(r3->getId(), timestamp,
                                      timestamp + ros::Duration(3.5)));
  EXPECT_TRUE(r2bs1_monitor->received(r1->getId(), timestamp,
                                      timestamp + ros::Duration(3.5)));
  EXPECT_TRUE(r2bs2_monitor->received(r3->getId(), timestamp,
                                      timestamp + ros::Duration(3.5)));
  EXPECT_TRUE(r3bs1_monitor->received(r1->getId(), timestamp,
                                      timestamp + ros::Duration(3.5)));
  EXPECT_TRUE(r3bs1_monitor->received(r2->getId(), timestamp,
                                      timestamp + ros::Duration(3.5)));
  EXPECT_TRUE(r1bs1_monitor->received(r2->getId(),
                                      timestamp + ros::Duration(1.5),
                                      timestamp + ros::Duration(2.5)));
  EXPECT_TRUE(r2bs1_monitor->received(r1->getId(),
                                      timestamp + ros::Duration(1.0),
                                      timestamp + ros::Duration(2.5)));
  EXPECT_TRUE(r3bs1_monitor->received(r1->getId(),
                                      timestamp + ros::Duration(1.0),
                                      timestamp + ros::Duration(2.5)));
  EXPECT_TRUE(r3bs1_monitor->received(r2->getId(),
                                      timestamp + ros::Duration(1.5),
                                      timestamp + ros::Duration(2.5)));
  EXPECT_FALSE(r1bs1_monitor->received(r3->getId(), timestamp,
                                       timestamp + ros::Duration(3.5)));
  EXPECT_FALSE(r1bs2_monitor->received(r2->getId(), timestamp,
                                       timestamp + ros::Duration(3.5)));
  EXPECT_FALSE(r2bs1_monitor->received(r3->getId(), timestamp,
                                       timestamp + ros::Duration(3.5)));
  EXPECT_FALSE(r2bs2_monitor->received(r1->getId(), timestamp,
                                       timestamp + ros::Duration(3.5)));
  EXPECT_FALSE(r1bs1_monitor->received(r2->getId(),
                                       timestamp + ros::Duration(2.51),
                                       timestamp + ros::Duration(3.5)));
  EXPECT_FALSE(r2bs1_monitor->received(r1->getId(),
                                       timestamp + ros::Duration(2.01),
                                       timestamp + ros::Duration(3.5)));
  EXPECT_FALSE(r3bs1_monitor->received(r1->getId(),
                                       timestamp + ros::Duration(2.01),
                                       timestamp + ros::Duration(3.5)));
  EXPECT_FALSE(r3bs1_monitor->received(r2->getId(),
                                       timestamp + ros::Duration(2.51),
                                       timestamp + ros::Duration(3.5)));
}

void init()
{
  t1.reset(new Task("t1", "task1"));
  t2.reset(new Task("t2", "task2"));
  r1_subject.reset(new nodes::InterRobotCommunicationSubject("r1/subject"));
  r2_subject.reset(new nodes::InterRobotCommunicationSubject("r2/subject"));
  r3_subject.reset(new nodes::InterRobotCommunicationSubject("r3/subject"));

  r1.reset(new Robot("r1", "robot 1", "r1"));
  r1->setBroadcastRate(ros::Rate(1.0));
  r1->setTimeoutDuration(ros::Duration(1.5));
  BehaviourSetPtr r1bs1(new BehaviourSet(r1, t1, ros::Duration(10.0)));
  r1bs1->init();
  r1bs1->setActivationThreshold(60);
  r1bs1->setAcquiescence(ros::Duration(0.75), ros::Duration(7.5));
  r1bs1->setImpatience(2.5);
  r1bs1_mb = r1bs1->getMotivationalBehaviour();
  r1_subject->registerObserver(r1bs1_mb->getInterRobotCommunication());
  r1->addBehaviourSet(r1bs1);
  BehaviourSetPtr r1bs2(new BehaviourSet(r1, t2, ros::Duration(5.0)));
  r1bs2->init();
  r1bs2->setActivationThreshold(80);
  r1bs2->setAcquiescence(ros::Duration(0.6), ros::Duration(6.0));
  r1bs2->setImpatience(1.0);
  r1bs2_mb = r1bs2->getMotivationalBehaviour();
  r1_subject->registerObserver(r1bs2_mb->getInterRobotCommunication());
  r1->addBehaviourSet(r1bs2);

  r2.reset(new Robot("r2", "robot 2", "r2"));
  r2->setBroadcastRate(ros::Rate(1.0));
  r2->setTimeoutDuration(ros::Duration(1.5));
  BehaviourSetPtr r2bs1(new BehaviourSet(r2, t1, ros::Duration(10.0)));
  r2bs1->init();
  r2bs1->setActivationThreshold(60);
  r2bs1->setAcquiescence(ros::Duration(0.75), ros::Duration(7.5));
  r2bs1->setImpatience(2.5);
  r2bs1_mb = r2bs1->getMotivationalBehaviour();
  r2_subject->registerObserver(r2bs1_mb->getInterRobotCommunication());
  r2->addBehaviourSet(r2bs1);
  BehaviourSetPtr r2bs2(new BehaviourSet(r2, t2, ros::Duration(5.0)));
  r2bs2->init();
  r2bs2->setActivationThreshold(80);
  r2bs2->setAcquiescence(ros::Duration(0.6), ros::Duration(6.0));
  r2bs2->setImpatience(1.0);
  r2bs2_mb = r2bs2->getMotivationalBehaviour();
  r2_subject->registerObserver(r2bs2_mb->getInterRobotCommunication());
  r2->addBehaviourSet(r2bs2);

  r3.reset(new Robot("r3", "robot 3", "r3"));
  r3->setBroadcastRate(ros::Rate(1.0));
  r3->setTimeoutDuration(ros::Duration(1.5));
  BehaviourSetPtr r3bs1(new BehaviourSet(r3, t1, ros::Duration(10.0)));
  r3bs1->init();
  r3bs1->setActivationThreshold(60);
  r3bs1->setAcquiescence(ros::Duration(0.75), ros::Duration(7.5));
  r3bs1->setImpatience(2.5);
  r3bs1_mb = r3bs1->getMotivationalBehaviour();
  r3_subject->registerObserver(r3bs1_mb->getInterRobotCommunication());
  r1->addBehaviourSet(r3bs1);
  BehaviourSetPtr r3bs2(new BehaviourSet(r3, t2, ros::Duration(5.0)));
  r3bs2->init();
  r3bs2->setActivationThreshold(80);
  r3bs2->setAcquiescence(ros::Duration(0.6), ros::Duration(6.0));
  r3bs2->setImpatience(1.0);
  r3bs2_mb = r3bs2->getMotivationalBehaviour();
  r3_subject->registerObserver(r3bs2_mb->getInterRobotCommunication());
  r3->addBehaviourSet(r3bs2);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "alliance_test_node");
  ros::NodeHandle nh;
  init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
