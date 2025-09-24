//
// Created by guanlin on 25-8-15.
//

#pragma once

// OCS2
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ipm/IpmSettings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_quadrotor/definitions.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_sqp/SqpSettings.h>

#include "quadrotor_interface/reference_manager/ReferenceManager.h"

namespace quadrotor_interface
{
class QuadrotorInterface : public ocs2::RobotInterface
{
public:
  QuadrotorInterface(const std::string & taskFile, const std::string & libraryFolder);
  ~QuadrotorInterface() override = default;

  void setupOptimalControlProblem(const std::string & taskFile, const std::string & libraryFolder);

  const ocs2::OptimalControlProblem & getOptimalControlProblem() const override
  {
    return *problemPtr_;
  }

  //  ocs2::sqp::Settings & sqpSettings() { return sqpSettings_; }
  ocs2::ddp::Settings & ddpSettings() { return ddpSettings_; }
  ocs2::mpc::Settings & mpcSettings() { return mpcSettings_; }
  //  ocs2::ipm::Settings & ipmSettings() { return ipmSettings_; }

  const vector_t & getInitialState() { return initialState_; }
  const ocs2::RolloutBase & getRollout() const { return *rolloutPtr_; }

  const ocs2::Initializer & getInitializer() const override { return *initializerPtr_; }
  std::shared_ptr<ocs2::ReferenceManagerInterface> getReferenceManagerPtr() const override
  {
    return referenceManagerPtr_;
  }

private:
  ocs2::ddp::Settings ddpSettings_;
  ocs2::mpc::Settings mpcSettings_;
  //  ocs2::sqp::Settings sqpSettings_;
  //  ocs2::ipm::Settings ipmSettings_;

  std::unique_ptr<ocs2::OptimalControlProblem> problemPtr_;
  std::shared_ptr<ocs2::ReferenceManager> referenceManagerPtr_;

  std::unique_ptr<ocs2::RolloutBase> rolloutPtr_;
  std::unique_ptr<ocs2::Initializer> initializerPtr_;

  vector_t initialState_{ocs2::quadrotor::STATE_DIM};
};
}  // namespace quadrotor_interface
