#include "controller/crab_controller/crab_state_machines/reorient.hpp"

#include "controller/crab_controller/crab_control_architecture.hpp"
#include "controller/crab_controller/crab_state_provider.hpp"
#include "controller/crab_controller/crab_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "util/interpolation.hpp"
#include "util/util.hpp"

Reorient::Reorient(const StateId state_id, PinocchioRobotSystem *robot,
                   CrabControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch) {
  util::PrettyConstructor(2, "Reorient");
  sp_ = CrabStateProvider::GetStateProvider();

  nominal_lfoot_iso_.setIdentity();
  nominal_rfoot_iso_.setIdentity();
  nominal_lhand_iso_.setIdentity();
  nominal_rhand_iso_.setIdentity();
}

Reorient::~Reorient() {
  if (min_jerk_curves_ != nullptr) delete min_jerk_curves_;
}

Eigen::Isometry3d GetEndEffectorTransform(Eigen::Isometry3d torso_iso,
                                          Eigen::Quaterniond target_quat) {
  // to rotate: torso.inverse(), then perform rotation as full transformation,
  // then transform torso
  Eigen::Isometry3d rotate(target_quat);
  return torso_iso * rotate * torso_iso.inverse();
}

void Reorient::FirstVisit() {
  std::cout << "crab_states::kReorient" << std::endl;
  state_machine_start_time_ = sp_->current_time_;

  Eigen::Vector3d init_com_pos = robot_->GetRobotComPos();
  Eigen::Matrix3d R_w_torso =
      robot_->GetLinkIsometry(crab_link::base_link).linear();
  Eigen::Quaterniond init_torso_quat(R_w_torso);

  Eigen::Quaterniond target_torso_quat =
      util::EulerZYXtoQuat(target_ori_[0], target_ori_[1], target_ori_[2]);

  Eigen::Vector3d com_ori_trans =
      target_ori_ - util::QuatToEulerZYX(init_torso_quat);

  ctrl_arch_->floating_base_tm_->InitializeFloatingBaseInterpolation(
      init_com_pos, init_com_pos, init_torso_quat, target_torso_quat,
      duration_);

  Eigen::Isometry3d foot_transform = GetEndEffectorTransform(
      robot_->GetLinkIsometry(crab_link::base_link), target_torso_quat);

  // Set current foot position as nominal (desired)
  nominal_lfoot_iso_ =
      foot_transform * robot_->GetLinkIsometry(crab_link::back_left__foot_link);
  nominal_rfoot_iso_ = foot_transform * robot_->GetLinkIsometry(
                                            crab_link::back_right__foot_link);
  nominal_lhand_iso_ = foot_transform * robot_->GetLinkIsometry(
                                            crab_link::front_left__foot_link);
  nominal_rhand_iso_ = foot_transform * robot_->GetLinkIsometry(
                                            crab_link::front_right__foot_link);

  // SetEndEffectorIso(R_w_torso, target_torso_quat, nominal_lfoot_iso_);
  // SetEndEffectorIso(R_w_torso, target_torso_quat, nominal_rfoot_iso_);
  // SetEndEffectorIso(R_w_torso, target_torso_quat, nominal_lhand_iso_);
  // SetEndEffectorIso(R_w_torso, target_torso_quat, nominal_rhand_iso_);

  // // get the vector from end effector to target from crab sensor data
  // Eigen::Vector3d lfoot_target_vector = target_ori_;
  // Eigen::Vector3d rfoot_target_vector = sp_->rfoot_target_vector_;
  // Eigen::Vector3d lhand_target_vector = sp_->lhand_target_vector_;
  // Eigen::Vector3d rhand_target_vector = sp_->rhand_target_vector_;

  // // translate foot toward landing object
  // Eigen::Isometry3d fin_lfoot_iso_ = nominal_lfoot_iso_;
  // Eigen::Isometry3d fin_rfoot_iso_ = nominal_rfoot_iso_;
  // Eigen::Isometry3d fin_lhand_iso_ = nominal_lhand_iso_;
  // Eigen::Isometry3d fin_rhand_iso_ = nominal_rhand_iso_;

  std::cout << "init_torso_quat:\n" << init_torso_quat << std::endl;
  std::cout << "target torso quat:\n" << target_torso_quat << std::endl;
  std::cout << "lhand_current:\n"
            << robot_->GetLinkIsometry(crab_link::front_right__foot_link)
                   .translation()
            << std::endl;
  std::cout << "lhand_target:\n"
            << nominal_lhand_iso_.translation() << std::endl;

  // Initialize interpolation
  ctrl_arch_->lf_SE3_tm_->InitializeSwingTrajectory(
      robot_->GetLinkIsometry(crab_link::back_left__foot_link),
      nominal_lfoot_iso_, nominal_lfoot_iso_.translation().z(), duration_);
  ctrl_arch_->rf_SE3_tm_->InitializeSwingTrajectory(
      robot_->GetLinkIsometry(crab_link::back_right__foot_link),
      nominal_rfoot_iso_, nominal_rfoot_iso_.translation().z(), duration_);
  ctrl_arch_->lh_SE3_tm_->InitializeSwingTrajectory(
      robot_->GetLinkIsometry(crab_link::front_left__foot_link),
      nominal_lhand_iso_, nominal_lhand_iso_.translation().z(), duration_);
  ctrl_arch_->rh_SE3_tm_->InitializeSwingTrajectory(
      robot_->GetLinkIsometry(crab_link::front_right__foot_link),
      nominal_rhand_iso_, nominal_rhand_iso_.translation().z(), duration_);
}

void Reorient::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  ctrl_arch_->floating_base_tm_->UpdateDesired(state_machine_time_);

  Eigen::Quaterniond cur_torso_quat(
      robot_->GetLinkIsometry(crab_link::base_link).linear());
  // std::cout << "cur_torso_quat:\n"
  //           << util::QuatToEulerXYZ(cur_torso_quat) << std::endl
  //           << std::endl;

  ctrl_arch_->lf_SE3_tm_->UpdateDesired(state_machine_time_);
  ctrl_arch_->rf_SE3_tm_->UpdateDesired(state_machine_time_);
  ctrl_arch_->lh_SE3_tm_->UpdateDesired(state_machine_time_);
  ctrl_arch_->rh_SE3_tm_->UpdateDesired(state_machine_time_);
}

void Reorient::LastVisit() {}

bool Reorient::EndOfState() { return false; }

StateId Reorient::GetNextState() { return crab_states::kReorient; }

void Reorient::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node["state_machine"]["reorient"], "target_orientation",
                        target_ori_);
    util::ReadParameter(node["state_machine"]["reorient"], "duration",
                        duration_);
  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
}
