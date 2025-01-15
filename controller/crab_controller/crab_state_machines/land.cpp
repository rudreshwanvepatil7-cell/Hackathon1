#include "controller/crab_controller/crab_state_machines/land.hpp"
#include "controller/crab_controller/crab_control_architecture.hpp"
#include "controller/crab_controller/crab_definition.hpp"
#include "controller/crab_controller/crab_state_provider.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
// 
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "planner/locomotion/dcm_planner/foot_step.hpp"

// Constructor 
Land::Land(const StateId state_id, PinocchioRobotSystem *robot,
           CrabControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch) {
  util::PrettyConstructor(2, "Land");

  sp_ = CrabStateProvider::GetStateProvider();
  nominal_lfoot_iso_.setIdentity();
  nominal_rfoot_iso_.setIdentity();
}


// First visit to the state
void Land::FirstVisit() {
  std::cout << "crab_states: kLand" << std::endl;
  state_machine_start_time_ = sp_->current_time_;

  // TODO set torso orientation to something meaningful
  Eigen::Vector3d init_com_pos = robot_->GetRobotComPos();
  Eigen::Matrix3d R_w_torso =
      robot_->GetLinkIsometry(crab_link::base_link).linear();
  Eigen::Quaterniond init_torso_quat(R_w_torso);
  double duration = 10.;
  //  ctrl_arch_->floating_base_tm_->InitializeFloatingBaseInterpolation(
  //      init_com_pos, init_com_pos,
  //      init_torso_quat, init_torso_quat, duration);

  // Set current foot position as nominal (desired)
  nominal_lfoot_iso_ = robot_->GetLinkIsometry(crab_link::back_left__foot_link);
  nominal_rfoot_iso_ =
      robot_->GetLinkIsometry(crab_link::back_right__foot_link);
  nominal_lhand_iso_ =
      robot_->GetLinkIsometry(crab_link::front_left__foot_link);
  nominal_rhand_iso_ =
      robot_->GetLinkIsometry(crab_link::front_right__foot_link);

  // rotate nominal foot towards landing object
  nominal_lfoot_iso_.rotate(Eigen::AngleAxisd(0.75, Eigen::Vector3d::UnitX()));
  nominal_rfoot_iso_.rotate(Eigen::AngleAxisd(-0.75, Eigen::Vector3d::UnitX()));
  nominal_lhand_iso_.rotate(Eigen::AngleAxisd(0.75, Eigen::Vector3d::UnitX()));
  nominal_rhand_iso_.rotate(Eigen::AngleAxisd(-0.75, Eigen::Vector3d::UnitX()));
  nominal_lfoot_iso_.rotate(Eigen::AngleAxisd(-0.5, Eigen::Vector3d::UnitY()));
  nominal_rfoot_iso_.rotate(Eigen::AngleAxisd(-0.5, Eigen::Vector3d::UnitY()));
  nominal_lhand_iso_.rotate(Eigen::AngleAxisd(-0.5, Eigen::Vector3d::UnitY()));
  nominal_rhand_iso_.rotate(Eigen::AngleAxisd(-0.5, Eigen::Vector3d::UnitY()));

  // Initialize interpolation
  ctrl_arch_->lf_SE3_tm_->InitializeSwingTrajectory(
      robot_->GetLinkIsometry(crab_link::back_left__foot_link),
      nominal_lfoot_iso_, nominal_lfoot_iso_.translation().z(), duration);
  ctrl_arch_->rf_SE3_tm_->InitializeSwingTrajectory(
      robot_->GetLinkIsometry(crab_link::back_right__foot_link),
      nominal_rfoot_iso_, nominal_rfoot_iso_.translation().z(), duration);
  ctrl_arch_->lh_SE3_tm_->InitializeSwingTrajectory(
      robot_->GetLinkIsometry(crab_link::front_left__foot_link),
      nominal_lhand_iso_, nominal_lhand_iso_.translation().z(), duration);
  ctrl_arch_->rh_SE3_tm_->InitializeSwingTrajectory(
      robot_->GetLinkIsometry(crab_link::front_right__foot_link),
      nominal_rhand_iso_, nominal_rhand_iso_.translation().z(), duration);
}

void Land::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  // com & torso ori task update
   ctrl_arch_->floating_base_tm_->UpdateDesired(state_machine_time_);

  // update foot pose task update
  if (b_use_fixed_foot_pos_) {
    ctrl_arch_->lf_SE3_tm_->UpdateDesired(state_machine_time_);
    ctrl_arch_->rf_SE3_tm_->UpdateDesired(state_machine_time_);
    ctrl_arch_->lh_SE3_tm_->UpdateDesired(state_machine_time_);
    ctrl_arch_->rh_SE3_tm_->UpdateDesired(state_machine_time_);
  } else {
    ctrl_arch_->lf_SE3_tm_->UseCurrent();
    ctrl_arch_->lh_SE3_tm_->UseCurrent();
    ctrl_arch_->rf_SE3_tm_->UseCurrent();
    ctrl_arch_->rh_SE3_tm_->UseCurrent();
  } 

  // update torso pose task update 
  // ctrl_arch_->floating_base_tm_->UpdateDesired(state_machine_time_); 

}

bool Land::EndOfState() { return false; }

void Land::LastVisit() {
  state_machine_time_ = 0.;

  if (sp_->b_use_base_height_)
    sp_->des_com_height_ = robot_->GetRobotComPos()[2];

  std::cout << "-----------------------------------------" << std::endl;
  std::cout << "des com height: " << sp_->des_com_height_ << std::endl;
  std::cout << "-----------------------------------------" << std::endl;

  Eigen::Isometry3d torso_iso = robot_->GetLinkIsometry(crab_link::base_link);
  FootStep::MakeHorizontal(torso_iso);
  sp_->rot_world_local_ = torso_iso.linear();
}

StateId Land::GetNextState() {}

void Land::SetParameters(const YAML::Node &node) {
  try {
    b_use_fixed_foot_pos_ = util::ReadParameter<bool>(
        node["state_machine"], "b_use_const_desired_foot_pos");
  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }
}
