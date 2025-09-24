//
// Created by guanlin on 25-8-20.
//

#pragma once

#include <geometry_msgs/Twist.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <memory>
#include <string>
#include <utility>

namespace quadrotor_interface
{
using ocs2::scalar_t;
using ocs2::vector_t;

class RosReferenceManager : public ocs2::ReferenceManagerDecorator
{
public:
  RosReferenceManager(std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr);
  ~RosReferenceManager() override = default;

  void subscribe(ros::NodeHandle & nodeHandle);
  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t & initState) override;

private:
  ::ros::Subscriber cmdVelSubscriber_;
  std::mutex cmdVelMutex_;
  std::atomic_bool cmdVelUpdated_;
  geometry_msgs::Twist cmdVel_;

  ::ros::Subscriber cmdHeightSubscriber_;
  std::mutex cmdHeightMutex_;
  std::atomic_bool cmdHeightUpdated_;
  std_msgs::Float64 cmdHeight_;
};
}  // namespace quadrotor_interface
