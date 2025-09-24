//
// Created by guanlin on 25-9-15.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <quadrotor_common/wrench_interface.h>

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
  void update(const ros::Time & time, const ros::Duration & period) override;

private:
  hardware_interface::QuadrotorWrenchHandle wrench_handle_;
};
}  // namespace quadrotor_controller
