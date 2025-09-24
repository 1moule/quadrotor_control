//
// Created by guanlin on 25-8-20.
//

#include "quadrotor_interface/reference_manager/ReferenceManager.h"

#include <ocs2_quadrotor/definitions.h>

namespace quadrotor_interface
{
RosReferenceManager::RosReferenceManager(
  std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr)
: ReferenceManagerDecorator(std::move(referenceManagerPtr))
{
}

void RosReferenceManager::preSolverRun(
  scalar_t initTime, scalar_t finalTime, const vector_t & initState)
{
  if (cmdVelUpdated_) {
    std::lock_guard<std::mutex> lock(cmdVelMutex_);
    cmdVelUpdated_ = false;

    ocs2::scalar_array_t timeTrajectory;
    ocs2::vector_array_t stateTrajectory;
    ocs2::vector_array_t inputTrajectory;
    scalar_t horizon = finalTime - initTime;

    const size_t sample = 20;
    const scalar_t dt = horizon / sample;
    for (size_t i = 0; i < sample + 1; ++i) {
      scalar_t time = dt * static_cast<scalar_t>(i);
      const vector_t targetState = [&]() {
        vector_t targetState = vector_t::Zero(ocs2::quadrotor::STATE_DIM);
        targetState(0) = initState(0) + cmdVel_.linear.x * time;
        targetState(1) = initState(1) + cmdVel_.linear.y * time;
        targetState(2) = cmdHeight_.data;
        //        targetState(3) = initState(3) + cmdVel_.angular.x * time;
        //        targetState(4) = initState(4) + cmdVel_.angular.y * time;
        //        targetState(5) = initState(5) + cmdVel_.angular.z * time;
        targetState(6) = cmdVel_.linear.x;
        targetState(7) = cmdVel_.linear.y;
        //        targetState(8) = cmdVel_.linear.z;
        //        targetState(9) = cmdVel_.angular.x;
        //        targetState(10) = cmdVel_.angular.y;
        //        targetState(11) = cmdVel_.angular.z;
        return targetState;
      }();
      timeTrajectory.push_back(initTime + time);
      stateTrajectory.push_back(targetState);
      inputTrajectory.emplace_back(vector_t::Zero(ocs2::quadrotor::INPUT_DIM));
    }
    referenceManagerPtr_->setTargetTrajectories({timeTrajectory, stateTrajectory, inputTrajectory});
  }
  referenceManagerPtr_->preSolverRun(initTime, finalTime, initState);
}

void RosReferenceManager::subscribe(ros::NodeHandle & nodeHandle)
{
  auto cmdVelCallback = [this](const geometry_msgs::Twist::ConstPtr & msg) {
    std::lock_guard<std::mutex> lock(cmdVelMutex_);
    cmdVelUpdated_ = true;
    cmdVel_ = *msg;
  };
  cmdVelSubscriber_ = nodeHandle.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelCallback);

  auto cmdHeightCallback = [this](const std_msgs::Float64::ConstPtr & msg) {
    std::lock_guard<std::mutex> lock(cmdHeightMutex_);
    cmdHeightUpdated_ = true;
    cmdHeight_ = *msg;
  };
  cmdHeightSubscriber_ =
    nodeHandle.subscribe<std_msgs::Float64>("/cmd_height", 1, cmdHeightCallback);
}

}  // namespace quadrotor_interface