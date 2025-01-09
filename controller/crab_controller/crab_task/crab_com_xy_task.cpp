#include "controller/crab_controller/crab_task/crab_com_xy_task.hpp"
#include "controller/crab_controller/crab_definition.hpp"
#include "controller/crab_controller/crab_state_provider.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "util/util.hpp"

#include <cmath>
#include <stdexcept>

#if B_USE_ZMQ
#include "controller/crab_controller/crab_data_manager.hpp"
#endif

CrabCoMXYTask::CrabCoMXYTask(PinocchioRobotSystem *robot)
    : Task(robot, 2), feedback_source_(feedback_source::kCoMFeedback) {
  util::PrettyConstructor(3, "CrabCoMXYTask");

  sp_ = CrabStateProvider::GetStateProvider();
}

CrabCoMXYTask::~CrabCoMXYTask() {}

void CrabCoMXYTask::UpdateOpCommand(const Eigen::Matrix3d &world_R_local) {
  Eigen::Vector2d com_xy_pos = robot_->GetRobotComPos().head<2>();
  Eigen::Vector2d com_xy_vel = sp_->com_vel_est_.head<2>();

  pos_ << com_xy_pos[0], com_xy_pos[1];
  vel_ << com_xy_vel[0], com_xy_vel[1];

  pos_err_ = des_pos_ - pos_;
  vel_err_ = des_vel_ - vel_;

  Eigen::Matrix2d local_R_world =
      world_R_local.transpose().topLeftCorner<2, 2>();
  // std::cout <<
  // "============================================================="
  //<< std::endl;
  // util::PrettyPrint(local_R_world, std::cout, "com xy original local rot
  // mat"); local_R_world.col(0).normalize(); local_R_world.col(1).normalize();
  // util::PrettyPrint(local_R_world, std::cout, "com xy normalized local rot
  // mat"); std::cout << "cross product result: "
  //<< local_R_world.col(0).dot(local_R_world.col(1)) << std::endl;

  //=============================================================
  // local com xy task data
  //=============================================================
  // TODO: notice that torso Rx, Ry need to be precisely controlled
  // TODO: notice that torso Rx, Ry need to be precisely controlled
  local_des_pos_ = local_R_world * des_pos_;
  local_pos_ = local_R_world * pos_;
  local_pos_err_ = local_R_world * pos_err_;

  local_des_vel_ = local_R_world * des_vel_;
  local_vel_ = local_R_world * vel_;
  local_vel_err_ = local_R_world * vel_err_;

  local_des_acc_ = local_R_world * des_acc_;

  if (feedback_source_ == feedback_source::kCoMFeedback) {
    //=============================================================
    // operational space command
    //=============================================================
    op_cmd_ = des_acc_ +
              local_R_world.transpose() * (kp_.cwiseProduct(local_pos_err_) +
                                           kd_.cwiseProduct(local_vel_err_));
  }
}

void CrabCoMXYTask::UpdateJacobian() {
  jacobian_ = robot_->GetComLinJacobian().topRows<2>();
}

void CrabCoMXYTask::UpdateJacobianDotQdot() {
  jacobian_dot_q_dot_ = robot_->GetComLinJacobianDotQdot().head<2>();
}

void CrabCoMXYTask::SetParameters(const YAML::Node &node,
                                  const WBC_TYPE wbc_type) {
  try {
    util::ReadParameter(node, "com_feedback_source", feedback_source_);

    if (wbc_type == WBC_TYPE::IHWBC) {
      if (feedback_source_ == feedback_source::kCoMFeedback)
        util::ReadParameter(node, "weight", weight_);
    }

    if (feedback_source_ == feedback_source::kCoMFeedback) {
      util::ReadParameter(node, "kp", kp_);
      util::ReadParameter(node, "kd", kd_);
    } else
      throw std::invalid_argument("No Matching CoM Feedback Source");

  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  } catch (const std::invalid_argument &ex) {
    std::cerr << "Error: " << ex.what() << " at file: [" << __FILE__ << "]"
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
}
