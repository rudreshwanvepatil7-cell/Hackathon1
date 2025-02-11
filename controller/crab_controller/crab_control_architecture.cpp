#include "controller/crab_controller/crab_control_architecture.hpp"

#include "controller/crab_controller/crab_controller.hpp"
#include "controller/crab_controller/crab_definition.hpp"
#include "controller/crab_controller/crab_state_machines/approach.hpp"
#include "controller/crab_controller/crab_state_machines/contact.hpp"
#include "controller/crab_controller/crab_state_machines/initialize.hpp"
#include "controller/crab_controller/crab_state_machines/reorient.hpp"
#include "controller/crab_controller/crab_state_provider.hpp"
#include "controller/crab_controller/crab_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/hand_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/task_hierarchy_manager.hpp"
#include "util/util.hpp"

#if B_USE_FOXGLOVE
#include "UI/foxglove/client/parameter_subscriber.hpp"
#endif

CrabControlArchitecture::CrabControlArchitecture(PinocchioRobotSystem *robot,
                                                 const YAML::Node &cfg)
    : ControlArchitecture(robot) {
  util::PrettyConstructor(1, "CrabControlArchitecture");

  sp_ = CrabStateProvider::GetStateProvider();

  // set starting state
  std::string test_env_name = util::ReadParameter<std::string>(cfg, "env");
  if (test_env_name == "pybullet") {
    // prev_loco_state_ = crab_states::kApproach;
    // loco_state_ = crab_states::kApproach;
    // prev_loco_state_ = crab_states::kReorient;
    // loco_state_ = crab_states::kReorient;
    prev_loco_state_ = crab_states::kInitialize;
    loco_state_ = crab_states::kInitialize;
  } else {
    // mujoco & hw
    prev_loco_state_ = crab_states::kInitialize;
    loco_state_ = crab_states::kInitialize;
  }

  //=============================================================
  // initialize task, contact, controller, planner
  //=============================================================
  tci_container_ = new CrabTCIContainer(robot_, cfg);
  controller_ = new CrabController(tci_container_, robot_, cfg);

  //=============================================================
  // trajectory Managers
  //=============================================================
  //  initialize kinematics manager
  floating_base_tm_ = new FloatingBaseTrajectoryManager(
      tci_container_->task_map_["com_xy_task"],
      tci_container_->task_map_["com_z_task"],
      tci_container_->task_map_["torso_ori_task"], robot_);

  lf_SE3_tm_ = new EndEffectorTrajectoryManager(
      tci_container_->task_map_["lf_pos_task"],
      tci_container_->task_map_["lf_ori_task"], robot_);
  rf_SE3_tm_ = new EndEffectorTrajectoryManager(
      tci_container_->task_map_["rf_pos_task"],
      tci_container_->task_map_["rf_ori_task"], robot_);
  lh_SE3_tm_ = new EndEffectorTrajectoryManager(
      tci_container_->task_map_["lh_pos_task"],
      tci_container_->task_map_["lh_ori_task"], robot_);
  rh_SE3_tm_ = new EndEffectorTrajectoryManager(
      tci_container_->task_map_["rh_pos_task"],
      tci_container_->task_map_["rh_ori_task"], robot_);

  //=============================================================
  // foot task hierarchy manager
  //=============================================================
  Eigen::VectorXd weight_at_contact, weight_at_swing;
  try {
    util::ReadParameter(cfg["wbc"]["task"]["foot_pos_task"], "weight",
                        weight_at_contact);
  } catch (const std::runtime_error &ex) {
    std::cerr << "Error reading foot pos task parameter [" << ex.what()
              << "] at file: [" << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  //  lf_pos_hm_ =
  //      new TaskHierarchyManager(tci_container_->task_map_["lf_pos_task"],
  //                               weight_at_contact, weight_at_swing);
  //  rf_pos_hm_ =
  //      new TaskHierarchyManager(tci_container_->task_map_["rf_pos_task"],
  //                               weight_at_contact, weight_at_swing);

  try {
    util::ReadParameter(cfg["wbc"]["task"]["foot_ori_task"], "weight",
                        weight_at_contact);
  } catch (const std::runtime_error &ex) {
    std::cerr << "Error reading foot ori task parameter [" << ex.what()
              << "] at file: [" << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  //  lf_ori_hm_ =
  //      new TaskHierarchyManager(tci_container_->task_map_["lf_ori_task"],
  //                               weight_at_contact, weight_at_swing);
  //  rf_ori_hm_ =
  //      new TaskHierarchyManager(tci_container_->task_map_["rf_ori_task"],
  //                               weight_at_contact, weight_at_swing);

  //=============================================================
  // hand task hierarchy manager
  //=============================================================
  Eigen::VectorXd weight_at_initial;
  try {
    util::ReadParameter(cfg["wbc"]["task"]["hand_pos_task"], "weight",
                        weight_at_initial);
  } catch (const std::runtime_error &ex) {
    std::cerr << "Error reading hand pos task parameter [" << ex.what()
              << "] at file: [" << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  //  lh_pos_hm_ =
  //      new TaskHierarchyManager(tci_container_->task_map_["lh_pos_task"],
  //                               weight_at_teleop_triggered,
  //                               weight_at_initial);
  //  rh_pos_hm_ =
  //      new TaskHierarchyManager(tci_container_->task_map_["rh_pos_task"],
  //                               weight_at_teleop_triggered,
  //                               weight_at_initial);
  try {
    util::ReadParameter(cfg["wbc"]["task"]["hand_ori_task"], "weight",
                        weight_at_initial);
  } catch (const std::runtime_error &ex) {
    std::cerr << "Error reading hand ori task parameter [" << ex.what()
              << "] at file: [" << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  //  lh_ori_hm_ =
  //      new TaskHierarchyManager(tci_container_->task_map_["lh_ori_task"],
  //                               weight_at_teleop_triggered,
  //                               weight_at_initial);
  //  rh_ori_hm_ =
  //      new TaskHierarchyManager(tci_container_->task_map_["rh_ori_task"],
  //                               weight_at_teleop_triggered,
  //                               weight_at_initial);

  //=============================================================
  // attach Foxglove Clients to control parameters
  //=============================================================
#if B_USE_FOXGLOVE
  param_map_int_ = {{{"n_steps", dcm_tm_->GetNumSteps()}}};
  param_map_double_ = {{{"t_ss", dcm_planner_->GetTssPtr()},
                        {"t_ds", dcm_planner_->GetTdsPtr()}}};
  param_map_hm_ = {{{"lf_pos_task", lf_pos_hm_},
                    {"rf_pos_task", rf_pos_hm_},
                    {"lf_ori_task", lf_ori_hm_},
                    {"rf_ori_task", rf_ori_hm_}}};
  param_subscriber_ =
      new FoxgloveParameterSubscriber(param_map_int_, param_map_double_,
                                      tci_container_->task_map_, param_map_hm_);
#endif

  //=============================================================
  // initialize state machines
  //=============================================================
  // Locomotion
  locomotion_state_machine_container_[crab_states::kInitialize] =
      new Initialize(crab_states::kInitialize, robot_, this);
  locomotion_state_machine_container_[crab_states::kInitialize]->SetParameters(
      cfg);

  locomotion_state_machine_container_[crab_states::kApproach] =
      new Approach(crab_states::kApproach, robot_, this);
  locomotion_state_machine_container_[crab_states::kApproach]->SetParameters(
      cfg);

  locomotion_state_machine_container_[crab_states::kContact] =
      new Contact(crab_states::kContact, robot_, this);
  locomotion_state_machine_container_[crab_states::kContact]->SetParameters(
      cfg);

  locomotion_state_machine_container_[crab_states::kReorient] =
      new Contact(crab_states::kReorient, robot_, this);
  locomotion_state_machine_container_[crab_states::kReorient]->SetParameters(
      cfg);
}

CrabControlArchitecture::~CrabControlArchitecture() {
  delete tci_container_;
  delete controller_;

  // tm
  //  delete floating_base_tm_;
  delete lf_SE3_tm_;
  delete rf_SE3_tm_;

  // hm
  //  delete lf_pos_hm_;
  //  delete lf_ori_hm_;
  //  delete rf_pos_hm_;
  //  delete rf_ori_hm_;

  // state machines
  delete locomotion_state_machine_container_[crab_states::kInitialize];
  delete locomotion_state_machine_container_[crab_states::kApproach];
  delete locomotion_state_machine_container_[crab_states::kContact];
  delete locomotion_state_machine_container_[crab_states::kReorient]; 

#if B_USE_FOXGLOVE
  delete param_subscriber_;
#endif
}

void CrabControlArchitecture::GetCommand(void *command) {
  if (b_loco_state_first_visit_) {
    locomotion_state_machine_container_[loco_state_]->FirstVisit();
    b_loco_state_first_visit_ = false;
  }

#if B_USE_FOXGLOVE
  param_subscriber_->UpdateParameters();
#endif

  // desired trajectory update in state machine
  locomotion_state_machine_container_[loco_state_]->OneStep();
  // get control command
  controller_->GetCommand(command);

  if (locomotion_state_machine_container_[loco_state_]->EndOfState()) {
    locomotion_state_machine_container_[loco_state_]->LastVisit();
    prev_loco_state_ = loco_state_;
    loco_state_ =
        locomotion_state_machine_container_[loco_state_]->GetNextState();
    b_loco_state_first_visit_ = true;
  }

#if B_USE_TELEOP
  if (manipulation_state_machine_container_[manip_state_]->EndOfState()) {
    manipulation_state_machine_container_[manip_state_]->LastVisit();
    prev_manip_state_ = manip_state_;
    manip_state_ =
        manipulation_state_machine_container_[manip_state_]->GetNextState();
    b_manip_state_first_visit_ = true;
  }
#endif
}
