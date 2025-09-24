//
// Created by guanlin on 25-8-15.
//

#include "quadrotor_interface/QuadrotorInterface.h"

#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_quadrotor/dynamics/QuadrotorSystemDynamics.h>

#include <iostream>
#include <string>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace quadrotor_interface
{
QuadrotorInterface::QuadrotorInterface(
  const std::string & taskFile, const std::string & libraryFolder)
{
  // check that task file exists
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath))
    std::cerr << "[QuadrotorInterface] Loading task file: " << taskFilePath << std::endl;
  else
    throw std::invalid_argument(
      "[QuadrotorInterface] Task file not found: " + taskFilePath.string());
  // create library folder if it does not exist
  boost::filesystem::path libraryFolderPath(libraryFolder);
  boost::filesystem::create_directories(libraryFolderPath);
  std::cerr << "[QuadrotorInterface] Generated library path: " << libraryFolderPath << std::endl;

  // Default initial condition
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
  std::cerr << "x_init:   " << initialState_.transpose() << std::endl;

  // DDP SQP MPC settings
  ddpSettings_ = ocs2::ddp::loadSettings(taskFile, "ddp");
  mpcSettings_ = ocs2::mpc::loadSettings(taskFile, "mpc");
  //  sqpSettings_ = ocs2::sqp::loadSettings(taskFile, "sqp");
  //  ipmSettings_ = ocs2::ipm::loadSettings(taskFile, "ipm");

  // OptimalControlProblem
  setupOptimalControlProblem(taskFile, libraryFolder);
}

void QuadrotorInterface::setupOptimalControlProblem(
  const std::string & taskFile, const std::string & libraryFolder)
{
  // Optimal control problem
  problemPtr_ = std::make_unique<ocs2::OptimalControlProblem>();

  // Reference Manager
  referenceManagerPtr_ = std::make_shared<ocs2::ReferenceManager>();

  // Dynamics
  auto quadrotorParameters = ocs2::quadrotor::loadSettings(taskFile, "QuadrotorParameters", true);
  std::unique_ptr<ocs2::SystemDynamicsBase> dynamicsPtr;
  dynamicsPtr = std::make_unique<ocs2::quadrotor::QuadrotorSystemDynamics>(quadrotorParameters);
  problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

  // Cost
  ocs2::matrix_t Q(ocs2::quadrotor::STATE_DIM, ocs2::quadrotor::STATE_DIM);
  ocs2::matrix_t R(ocs2::quadrotor::INPUT_DIM, ocs2::quadrotor::INPUT_DIM);
  ocs2::matrix_t Qf(ocs2::quadrotor::STATE_DIM, ocs2::quadrotor::STATE_DIM);
  ocs2::loadData::loadEigenMatrix(taskFile, "Q", Q);
  ocs2::loadData::loadEigenMatrix(taskFile, "R", R);
  ocs2::loadData::loadEigenMatrix(taskFile, "Q_final", Qf);

  std::cerr << "Q:  \n" << Q << "\n";
  std::cerr << "R:  \n" << R << "\n";
  std::cerr << "Q_final:\n" << Qf << "\n";

  problemPtr_->costPtr->add("cost", std::make_unique<ocs2::QuadraticStateInputCost>(Q, R));
  //  problemPtr_->finalCostPtr->add("finalCost", std::make_unique<ocs2::QuadraticStateCost>(Qf));

  // Rollout
  auto rolloutSettings = ocs2::rollout::loadSettings(taskFile, "rollout");
  rolloutPtr_ =
    std::make_unique<ocs2::TimeTriggeredRollout>(*problemPtr_->dynamicsPtr, rolloutSettings);

  // Initialization
  vector_t initialInput = vector_t::Zero(ocs2::quadrotor::INPUT_DIM);
  initialInput(0) = quadrotorParameters.quadrotorMass_ * quadrotorParameters.gravity_;
  initializerPtr_.reset(new ocs2::OperatingPoints(initialState_, initialInput));
}
}  // namespace quadrotor_interface
