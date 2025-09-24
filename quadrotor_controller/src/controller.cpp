//
// Created by guanlin on 25-9-15.
//

#include "quadrotor_controller/controller.h"

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

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

  stateEstimate_ = std::make_shared<quadrotor_estimation::FromTopicStateEstimate>();

  // Initialize OCS2
  std::string taskFile;
  std::string libFolder;
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/libFolder", libFolder);
  quadrotor_interface_ =
    std::make_shared<quadrotor_interface::QuadrotorInterface>(taskFile, libFolder);
  setupMpc(controller_nh);
  setupMrt();

  return true;
}

void QuadrotorController::starting(const ros::Time & time)
{
  // Initial state
  currentObservation_.state.setZero(ocs2::quadrotor::STATE_DIM);
  updateEstimation(time, ros::Duration(0.001));
  currentObservation_.input.setZero(ocs2::quadrotor::INPUT_DIM);

  ocs2::TargetTrajectories target_trajectories(
    {currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(quadrotor_interface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  mpcRunning_ = true;
}

void QuadrotorController::stopping(const ros::Time & time)
{
  mpcRunning_ = false;
  geometry_msgs::Wrench cmd;
  cmd.force.x = 0;
  cmd.force.y = 0;
  cmd.force.z = 0;
  cmd.torque.x = 0;
  cmd.torque.y = 0;
  cmd.torque.z = 0;
  wrench_handle_.setCommand(cmd);
}

void QuadrotorController::updateEstimation(const ros::Time & time, const ros::Duration period)
{
  measuredRbdState_ = stateEstimate_->update(time, period);
  currentObservation_.time += period.toSec();
  currentObservation_.state = measuredRbdState_;
  currentObservation_.state[3] = measuredRbdState_[5];
  currentObservation_.state[5] = measuredRbdState_[3];
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
}

void QuadrotorController::update(const ros::Time & time, const ros::Duration & period)
{
  // State Estimate
  updateEstimation(time, period);

  // Update the current state of the system
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();

  // Evaluate the current policy
  ocs2::vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(
    currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput,
    plannedMode);

  geometry_msgs::Wrench cmd;
  cmd.force.x = 0;
  cmd.force.y = 0;
  cmd.force.z = optimizedInput[0];
  cmd.torque.x = optimizedInput[1];
  cmd.torque.y = optimizedInput[2];
  cmd.torque.z = optimizedInput[3];
  wrench_handle_.setCommand(cmd);

  currentObservation_.input = optimizedInput;
  observationPublisher_.publish(
    ocs2::ros_msg_conversions::createObservationMsg(currentObservation_));
}

void QuadrotorController::setupMpc(ros::NodeHandle & nh)
{
  const std::string robotName = "quadrotor";
  mpc_ = std::make_shared<ocs2::GaussNewtonDDP_MPC>(
    quadrotor_interface_->mpcSettings(), quadrotor_interface_->ddpSettings(),
    quadrotor_interface_->getRollout(), quadrotor_interface_->getOptimalControlProblem(),
    quadrotor_interface_->getInitializer());
  auto rosReferenceManagerPtr = std::make_shared<quadrotor_interface::RosReferenceManager>(
    quadrotor_interface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);

  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  observationPublisher_ =
    nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
}

void QuadrotorController::setupMrt()
{
  mpcMrtInterface_ = std::make_shared<ocs2::MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&quadrotor_interface_->getRollout());
  mpcTimer_.reset();

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      try {
        ocs2::executeAndSleep(
          [&]() {
            if (mpcRunning_) {
              mpcTimer_.startTimer();
              mpcMrtInterface_->advanceMpc();
              mpcTimer_.endTimer();
            }
          },
          quadrotor_interface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception & e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  ocs2::setThreadPriority(50, mpcThread_);
}
}  // namespace quadrotor_controller
PLUGINLIB_EXPORT_CLASS(
  quadrotor_controller::QuadrotorController, controller_interface::ControllerBase)
