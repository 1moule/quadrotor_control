//
// Created by guanlin on 25-9-15.
//

#include "quadrotor_controller/controller.h"

#include <pluginlib/class_list_macros.hpp>

namespace quadrotor_controller
{
bool QuadrotorController::init(
  hardware_interface::RobotHW * robot_hw, ros::NodeHandle & root_nh,
  ros::NodeHandle & controller_nh)
{
  // Hardware interface
  auto * wrenchInterface = robot_hw->get<hardware_interface::QuadrotorWrenchInterface>();
  wrench_handle_ = wrenchInterface->getHandle("wrench");
  return true;
}

void QuadrotorController::update(const ros::Time & time, const ros::Duration & period)
{
  geometry_msgs::Wrench cmd;
  cmd.force.x = 0;
  cmd.force.y = 0;
  cmd.force.z = 1.;
  cmd.torque.x = 0;
  cmd.torque.y = 0;
  cmd.torque.z = 0;
  wrench_handle_.setCommand(cmd);
}

}  // namespace quadrotor_controller
PLUGINLIB_EXPORT_CLASS(
  quadrotor_controller::QuadrotorController, controller_interface::ControllerBase)
