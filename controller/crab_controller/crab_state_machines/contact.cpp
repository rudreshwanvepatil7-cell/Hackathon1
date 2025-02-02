#include "controller/crab_controller/crab_state_machines/contact.hpp"

#include "controller/crab_controller/crab_control_architecture.hpp"
#include "controller/crab_controller/crab_state_provider.hpp"
#include "controller/crab_controller/crab_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "util/interpolation.hpp"

Contact::Contact(const StateId state_id, PinocchioRobotSystem *robot,
                 CrabControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot),
      ctrl_arch_(ctrl_arch),
      b_stay_here_(false),
      wait_time_(0.),
      min_jerk_curves_(nullptr) {
  util::PrettyConstructor(2, "Contact");

  sp_ = CrabStateProvider::GetStateProvider();
  target_joint_pos_ = Eigen::VectorXd::Zero(robot_->NumActiveDof());
  init_joint_pos_ = Eigen::VectorXd::Zero(robot_->NumActiveDof());
}

Contact::~Contact() {
  if (min_jerk_curves_ != nullptr) delete min_jerk_curves_;
}

void Contact::FirstVisit() {
  std::cout << "crab_states::kContact" << std::endl;
  state_machine_start_time_ = sp_->current_time_;
}

void Contact::OneStep() {
  // TODO: Implement
}

void Contact::LastVisit() {}

bool Contact::EndOfState() { return false; }

StateId Contact::GetNextState() {}

void Contact::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node["state_machine"]["initialize"], "init_duration",
                        end_time_);
    util::ReadParameter(node["state_machine"]["initialize"], "target_joint_pos",
                        target_joint_pos_);
    //    sp_->nominal_jpos_ = target_joint_pos_; // set nominal jpos
    util::ReadParameter(node["state_machine"]["initialize"],
                        "b_only_joint_pos_control", b_stay_here_);
    util::ReadParameter(node["state_machine"]["initialize"], "wait_time",
                        wait_time_);

  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
}
