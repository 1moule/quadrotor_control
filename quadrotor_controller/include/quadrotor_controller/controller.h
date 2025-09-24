//
// Created by guanlin on 25-9-15.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_msgs/mpc_observation.h>
#include <quadrotor_common/wrench_interface.h>
#include <quadrotor_interface/QuadrotorInterface.h>

namespace quadrotor_controller
{

class QuadrotorController : public controller_interface::MultiInterfaceController<
                              hardware_interface::QuadrotorWrenchInterface>
{
public:
  QuadrotorController() = default;
  bool init(
    hardware_interface::RobotHW * robot_hw, ros::NodeHandle & root_nh,
    ros::NodeHandle & controller_nh) override;
  void starting(const ros::Time & time) override;
  void update(const ros::Time & time, const ros::Duration & period) override;

private:
  void setupMpc(ros::NodeHandle & nh);
  void setupMrt();
  void updateEstimation(const ros::Time & time, const ros::Duration period);

  hardware_interface::QuadrotorWrenchHandle wrench_handle_;

  //mpc
  // Interface
  std::shared_ptr<quadrotor_interface::QuadrotorInterface> quadrotor_interface_;

  // State Estimation
  ocs2::SystemObservation currentObservation_;

  // MPC
  std::shared_ptr<ocs2::MPC_BASE> mpc_;
  std::shared_ptr<ocs2::MPC_MRT_Interface> mpcMrtInterface_;

  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  ocs2::benchmark::RepeatedTimer mpcTimer_;

  ros::Publisher observationPublisher_;
};
}  // namespace quadrotor_controller
