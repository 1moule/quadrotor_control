//
// Created by guanlin on 25-9-23.
//

#pragma once

#include <geometry_msgs/Wrench.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

#include <string>

namespace hardware_interface
{
class QuadrotorWrenchHandle
{
public:
  QuadrotorWrenchHandle() = default;
  QuadrotorWrenchHandle(const std::string & name, geometry_msgs::Wrench * wrench_cmd)
  : name_(name), wrench_cmd_(wrench_cmd)
  {
  }

  void setCommand(const geometry_msgs::Wrench & cmd)
  {
    assert(wrench_cmd_);
    *wrench_cmd_ = cmd;
  }

  geometry_msgs::Wrench getCommand() const
  {
    assert(wrench_cmd_);
    return *wrench_cmd_;
  }

  std::string getName() const { return name_; }

private:
  std::string name_;
  geometry_msgs::Wrench * wrench_cmd_{nullptr};
};

class QuadrotorWrenchInterface
: public hardware_interface::HardwareResourceManager<QuadrotorWrenchHandle>
{
};

}  // namespace hardware_interface