#include "controller/crab_controller/crab_state_machines/reorient.hpp"

#include "controller/crab_controller/crab_control_architecture.hpp"
#include "controller/crab_controller/crab_state_provider.hpp"
#include "controller/crab_controller/crab_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "util/interpolation.hpp"

Reorient::Reorient(const StateId state_id, PinocchioRobotSystem *robot,
                       CrabControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot),
      ctrl_arch_(ctrl_arch),
      b_stay_here_(false),
      wait_time_(0.),
      min_jerk_curves_(nullptr) {
  util::PrettyConstructor(2, "Reorient");

  sp_ = CrabStateProvider::GetStateProvider();
  target_joint_pos_ = Eigen::VectorXd::Zero(robot_->NumActiveDof());
  init_joint_pos_ = Eigen::VectorXd::Zero(robot_->NumActiveDof());
}

Reorient::~Reorient() {
  if (min_jerk_curves_ != nullptr) delete min_jerk_curves_;
}

void Reorient::FirstVisit() {
  std::cout << "crab_states::kReorient" << std::endl;
  state_machine_start_time_ = sp_->current_time_;
  init_joint_pos_ = robot_->GetJointPos();
  min_jerk_curves_ = new MinJerkCurveVec(
      init_joint_pos_, Eigen::VectorXd::Zero(init_joint_pos_.size()),
      Eigen::VectorXd::Zero(init_joint_pos_.size()), target_joint_pos_,
      Eigen::VectorXd::Zero(target_joint_pos_.size()),
      Eigen::VectorXd::Zero(target_joint_pos_.size()),
      end_time_);  // min jerk curve initialization
}

void Reorient::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  Eigen::VectorXd des_joint_pos =
      Eigen::VectorXd::Zero(target_joint_pos_.size());
  Eigen::VectorXd des_joint_vel =
      Eigen::VectorXd::Zero(target_joint_pos_.size());
  Eigen::VectorXd des_joint_acc =
      Eigen::VectorXd::Zero(target_joint_pos_.size());

  if (min_jerk_curves_ == nullptr)
    throw std::runtime_error(
        "Reorient MinJerkCurve in Reorient StateMachine");

  for (unsigned int i(0); i < target_joint_pos_.size(); ++i) {
    des_joint_pos = min_jerk_curves_->Evaluate(state_machine_time_);
    des_joint_vel =
        min_jerk_curves_->EvaluateFirstDerivative(state_machine_time_);
    des_joint_acc =
        min_jerk_curves_->EvaluateSecondDerivative(state_machine_time_);
  }
  ctrl_arch_->tci_container_->task_map_["joint_task"]->UpdateDesired(
      des_joint_pos, des_joint_vel, des_joint_acc);
}

void Reorient::LastVisit() {}

// bool Reorient::EndOfState() {
//   return !b_stay_here_ && (state_machine_time_ >= end_time_ + wait_time_);
// }
bool Reorient::EndOfState() { return false; }

// StateId Reorient::GetNextState() { return crab_states::kApproach; }
StateId Reorient::GetNextState() { return crab_states::kReorient; }

void Reorient::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node["state_machine"]["Reorient"], "init_duration",
                        end_time_);
    util::ReadParameter(node["state_machine"]["Reorient"], "target_joint_pos",
                        target_joint_pos_);
    //    sp_->nominal_jpos_ = target_joint_pos_; // set nominal jpos
    util::ReadParameter(node["state_machine"]["Reorient"],
                        "b_only_joint_pos_control", b_stay_here_);
    util::ReadParameter(node["state_machine"]["Reorient"], "wait_time",
                        wait_time_);

  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
}
