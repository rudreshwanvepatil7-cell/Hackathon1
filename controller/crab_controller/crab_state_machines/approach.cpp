#include "controller/crab_controller/crab_state_machines/approach.hpp"

#include "controller/crab_controller/crab_control_architecture.hpp"
#include "controller/crab_controller/crab_definition.hpp"
#include "controller/crab_controller/crab_state_provider.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
//
#include "controller/crab_controller/crab_tci_container.hpp"  // Include the header file for CrabTCIContainer
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "planner/locomotion/dcm_planner/foot_step.hpp"
#include "util/util.hpp"

// Constructor
Approach::Approach( const StateId state_id, 
                    PinocchioRobotSystem *robot,
                    CrabControlArchitecture *ctrl_arch )
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch) 
{
  util::PrettyConstructor(2, "Approach");
  sp_ = CrabStateProvider::GetStateProvider();
  nominal_lfoot_iso_.setIdentity();
  nominal_rfoot_iso_.setIdentity();
}

// Function to compute and set the rotation matrix
void SetRotationDCM( Eigen::Vector3d LIMB_target_vector, 
                     Eigen::Isometry3d & nominal_LIMB_iso_ ) 
{
  // Eigen::Vector3d z_axis = - LIMB_target_vector.normalized();
  Eigen::Vector3d z_axis(-1, 0, 0); 
  Eigen::Vector3d x_axis(0, 0, 1); 
  Eigen::Vector3d y_axis(0, 1, 0); 

  std::cout << "x_axis = " << x_axis << std::endl; 
  std::cout << "y_axis = " << y_axis << std::endl; 
  std::cout << "z_axis = " << z_axis << std::endl; 

  Eigen::Matrix3d rot_matrix; 
  rot_matrix.col(0) = x_axis; 
  rot_matrix.col(1) = y_axis; 
  rot_matrix.col(2) = z_axis; 

  nominal_LIMB_iso_.linear() = rot_matrix; 
} 

// First visit to the state
void Approach::FirstVisit() 
{
  std::cout << "crab_states: kApproach" << std::endl;
  state_machine_start_time_ = sp_->current_time_;

  // TODO set torso orientation to something meaningful
  Eigen::Vector3d init_com_pos = robot_->GetRobotComPos();
  Eigen::Matrix3d R_w_torso =
      robot_->GetLinkIsometry(crab_link::base_link).linear();
  Eigen::Quaterniond init_torso_quat(R_w_torso);
  double duration = 30.;

  // set target torso orientation
  Eigen::Quaterniond target_torso_quat = util::EulerZYXtoQuat(1.57/4, 0., 0.); 

  std::cout << "set target_torso_quat = " << target_torso_quat.coeffs() << std::endl; 

  // Eigen::Quaterniond target_torso_quat = init_torso_quat;

  // // get rotation matrix from body_target_iso and turn into quaternion 
  // Eigen::Vector3d body_target_vector = sp_->body_target_vector_; 
  // Eigen::Isometry3d body_target_iso  = robot_->GetLinkIsometry(crab_link::base_link);  
  // SetRotationDCM( body_target_vector, body_target_iso ); 
  // Eigen::Quaterniond target_torso_quat = Eigen::Quaterniond(body_target_iso.linear()); 

  ctrl_arch_->floating_base_tm_->InitializeFloatingBaseInterpolation(
      init_com_pos, init_com_pos, init_torso_quat, target_torso_quat, duration);

  std::cout << "\n\n init_torso_quat = \n" << init_torso_quat.coeffs() << std::endl; 
  std::cout << "\n\n target_torso_quat = \n" << target_torso_quat.coeffs() << std::endl; 

  // Set current foot position as nominal (desired)
  nominal_lfoot_iso_ = robot_->GetLinkIsometry(crab_link::back_left__foot_link);
  nominal_rfoot_iso_ = robot_->GetLinkIsometry(crab_link::back_right__foot_link);
  nominal_lhand_iso_ = robot_->GetLinkIsometry(crab_link::front_left__foot_link);
  nominal_rhand_iso_ = robot_->GetLinkIsometry(crab_link::front_right__foot_link); 

  // get the vector from end effector to target from crab sensor data 
  Eigen::Vector3d lfoot_target_vector = sp_->lfoot_target_vector_; 
  Eigen::Vector3d rfoot_target_vector = sp_->rfoot_target_vector_; 
  Eigen::Vector3d lhand_target_vector = sp_->lhand_target_vector_; 
  Eigen::Vector3d rhand_target_vector = sp_->rhand_target_vector_; 

  // // rotate nominal foot so that the z axis aligns with the lfoot_target_vector 
  // SetRotationDCM( lfoot_target_vector, nominal_lfoot_iso_ ); 
  // SetRotationDCM( rfoot_target_vector, nominal_rfoot_iso_ ); 
  // SetRotationDCM( lhand_target_vector, nominal_lhand_iso_ ); 
  // SetRotationDCM( rhand_target_vector, nominal_rhand_iso_ ); 

  // rotate nominal foot towards landing object
  // nominal_lfoot_iso_.rotate(Eigen::AngleAxisd(0.75,  Eigen::Vector3d::UnitX()));
  // nominal_rfoot_iso_.rotate(Eigen::AngleAxisd(-0.75, Eigen::Vector3d::UnitX()));
  // nominal_lhand_iso_.rotate(Eigen::AngleAxisd(0.75,  Eigen::Vector3d::UnitX()));
  // nominal_rhand_iso_.rotate(Eigen::AngleAxisd(-0.75, Eigen::Vector3d::UnitX()));

  // nominal_lfoot_iso_.rotate(Eigen::AngleAxisd(-0.5,  Eigen::Vector3d::UnitY()));
  // nominal_rfoot_iso_.rotate(Eigen::AngleAxisd(-0.5,  Eigen::Vector3d::UnitY()));
  // nominal_lhand_iso_.rotate(Eigen::AngleAxisd(-0.5,  Eigen::Vector3d::UnitY()));
  // nominal_rhand_iso_.rotate(Eigen::AngleAxisd(-0.5,  Eigen::Vector3d::UnitY())); 

  // translate foot toward landing object 
  Eigen::Isometry3d fin_lfoot_iso_ = nominal_lfoot_iso_; 
  Eigen::Isometry3d fin_rfoot_iso_ = nominal_rfoot_iso_; 
  Eigen::Isometry3d fin_lhand_iso_ = nominal_lhand_iso_; 
  Eigen::Isometry3d fin_rhand_iso_ = nominal_rhand_iso_; 

  // fin_lfoot_iso_.translation() += lfoot_target_vector; 
  // fin_rfoot_iso_.translation() += rfoot_target_vector; 
  // fin_lhand_iso_.translation() += lhand_target_vector; 
  // fin_rhand_iso_.translation() += rhand_target_vector; 

  std::cout << "nominal lfoot iso = " << nominal_lfoot_iso_.translation() << std::endl;
  std::cout << "fin lfoot iso = " << fin_lfoot_iso_.translation() << std::endl; 

  // Initialize interpolation
  // ctrl_arch_->lf_SE3_tm_->InitializeSwingTrajectory(
  //     robot_->GetLinkIsometry(crab_link::back_left__foot_link),
  //     nominal_lfoot_iso_, 
  //     fin_lfoot_iso_.translation().z(), 
  //     duration);
  // ctrl_arch_->rf_SE3_tm_->InitializeSwingTrajectory(
  //     robot_->GetLinkIsometry(crab_link::back_right__foot_link),
  //     nominal_rfoot_iso_, 
  //     fin_rfoot_iso_.translation().z(), 
  //     duration);
  // ctrl_arch_->lh_SE3_tm_->InitializeSwingTrajectory(
  //     robot_->GetLinkIsometry(crab_link::front_left__foot_link),
  //     nominal_lhand_iso_, 
  //     fin_lhand_iso_.translation().z(), 
  //     duration);
  // ctrl_arch_->rh_SE3_tm_->InitializeSwingTrajectory(
  //     robot_->GetLinkIsometry(crab_link::front_right__foot_link),
  //     nominal_rhand_iso_, 
  //     fin_rhand_iso_.translation().z(), 
  //     duration);

  // std::cout << "swing height lfoot trajectory = " << nominal_lfoot_iso_.translation().z() << std::endl; 
}

void Approach::OneStep() 
{
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;
  state_machine_time_ = std::min(state_machine_time_, 10.0); 

  // Eigen::Matrix3d R_w_torso =
  //   robot_->GetLinkIsometry(crab_link::base_link).linear();
  // Eigen::Quaterniond current_torso_quat(R_w_torso);
  // std::cout << "current_torso_quat = " << current_torso_quat.coeffs() << std::endl; 

  // com & torso ori task update
  ctrl_arch_->floating_base_tm_->UpdateDesired(state_machine_time_);

  // update foot pose task update
  // if (b_use_fixed_foot_pos_) {
  //   ctrl_arch_->lf_SE3_tm_->UpdateDesired(state_machine_time_);
  //   ctrl_arch_->rf_SE3_tm_->UpdateDesired(state_machine_time_);
  //   ctrl_arch_->lh_SE3_tm_->UpdateDesired(state_machine_time_);
  //   ctrl_arch_->rh_SE3_tm_->UpdateDesired(state_machine_time_);
  // } else {
  //   ctrl_arch_->lf_SE3_tm_->UseCurrent();
  //   ctrl_arch_->lh_SE3_tm_->UseCurrent();
  //   ctrl_arch_->rf_SE3_tm_->UseCurrent();
  //   ctrl_arch_->rh_SE3_tm_->UseCurrent();
  // }

}

bool Approach::EndOfState() {
  return sp_->b_lh_contact_ || sp_->b_rh_contact_ || sp_->b_lf_contact_ ||
         sp_->b_rf_contact_;
}

void Approach::LastVisit() 
{
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

// Comment out for now
// StateId Approach::GetNextState() { return crab_states::kContact; }
StateId Approach::GetNextState() { return crab_states::kReorient; }

void Approach::SetParameters(const YAML::Node &node) 
{
  try {
    b_use_fixed_foot_pos_ = util::ReadParameter<bool>(
        node["state_machine"], "b_use_const_desired_foot_pos");
  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }
}

