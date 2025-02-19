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
void SetRotationDCM( Eigen::Vector3d target_vector, 
                     Eigen::Isometry3d & body_iso ) 
{ 
  // Get current body orientation
  Eigen::Matrix3d R_current = body_iso.linear();
  Eigen::Vector3d current_neg_z = -R_current.col(2);  // Current -Z axis
  
  // Normalize target vector
  Eigen::Vector3d target = target_vector.normalized();
  
  // Compute rotation axis and angle
  Eigen::Vector3d rotation_axis = current_neg_z.cross(target);
  
  if (rotation_axis.norm() > 1e-6) {  // Check if vectors aren't parallel
      rotation_axis.normalize();
      double rotation_angle = std::acos(current_neg_z.dot(target));
      
      // Create rotation quaternion
      Eigen::Quaterniond q = Eigen::Quaterniond(
          Eigen::AngleAxisd(rotation_angle, rotation_axis)); 

      // convert the quaternion to a rotation matrix 
      Eigen::Matrix3d R_new = q.toRotationMatrix();
          
      // Final rotation = new rotation * current rotation
      Eigen::Matrix3d final_R = R_new * R_current ;
      
      // Set the rotation in the isometry
      body_iso.linear() = final_R;
      
      // Debug output
      std::cout << "Current -Z: " << current_neg_z.transpose() << std::endl;
      std::cout << "Target: " << target.transpose() << std::endl;
      std::cout << "Rotation axis: " << rotation_axis.transpose() << std::endl;
      std::cout << "Rotation angle (deg): " << rotation_angle * 180/M_PI << std::endl;
  }
  else {
      // Vectors are parallel - no rotation needed or 180-degree rotation
      if (current_neg_z.dot(target) < 0) {
          // 180-degree rotation needed around any perpendicular axis
          Eigen::Vector3d perp = current_neg_z.unitOrthogonal();
          Eigen::Matrix3d final_R = Eigen::AngleAxisd(M_PI, perp).toRotationMatrix() * R_current;
          body_iso.linear() = final_R;
      }
  }
} 

// First visit to the state
void Approach::FirstVisit() 
{
  // StateMachine::FirstVisit();
  
  // // Initialize PID controller with gains
  // orientation_pid_ = new PIDController(1.0, 0.0, 0.1);
  prev_time_ = sp_->current_time_;
  
  std::cout << "crab_states: kApproach" << std::endl;
  state_machine_start_time_ = sp_->current_time_;
  double duration = 30.;

  // TODO set torso orientation to something meaningful
  Eigen::Vector3d init_com_pos = robot_->GetRobotComPos();
  Eigen::Matrix3d R_w_torso =
      robot_->GetLinkIsometry(crab_link::base_link).linear();
  Eigen::Quaterniond init_torso_quat(R_w_torso);

  // // set target torso orientation
  // Eigen::Quaterniond target_torso_quat = util::EulerZYXtoQuat(1.57, 0., 0.); 
  // Eigen::Quaterniond target_torso_quat = init_torso_quat;

  // Get target vector and current body pose
  Eigen::Vector3d body_target_vector = sp_->body_target_vector_; 

  // Compute rotation to align -Z with target
  Eigen::Isometry3d body_target_iso = robot_->GetLinkIsometry(crab_link::base_link);
  SetRotationDCM(body_target_vector, body_target_iso);

  // Convert to quaternion for the controller
  Eigen::Quaterniond target_torso_quat(body_target_iso.linear());
  std::cout << "Target quaternion: " << target_torso_quat << std::endl;

  // Debug output
  std::cout << "Target vector: " << body_target_vector.transpose() << std::endl;
  std::cout << "Resulting -Z axis: " << -body_target_iso.linear().col(2).transpose() << std::endl;

  ctrl_arch_->floating_base_tm_->InitializeFloatingBaseInterpolation(
      init_com_pos, init_com_pos, init_torso_quat, target_torso_quat, duration);

  // std::cout << "body_target_vector = \n" << body_target_vector << std::endl; 
  // // std::cout << "\n\n init_torso_quat = \n" << init_torso_quat.coeffs() << std::endl; 
  // std::cout << "\n\n target_torso_quat = \n" << target_torso_quat.coeffs() << std::endl; 
}

void Approach::OneStep() 
{
  // Calculate dt
  double curr_time = sp_->current_time_;
  double dt = curr_time - prev_time_;
  prev_time_ = curr_time; 

  // com & torso ori task update
  ctrl_arch_->floating_base_tm_->UpdateDesired(state_machine_time_);

  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;
  // state_machine_time_ = std::min(state_machine_time_, 10.0); 

}

bool Approach::EndOfState() {
  return sp_->b_lh_contact_ || sp_->b_rh_contact_ || sp_->b_lf_contact_ ||
         sp_->b_rf_contact_;
}

void Approach::LastVisit() 
{
  delete orientation_pid_;
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

