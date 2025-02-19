#include <gtest/gtest.h>

#include "controller/crab_controller/crab_definition.hpp"
#include "controller/crab_controller/crab_tci_container.hpp"
#include "controller/whole_body_controller/contact.hpp"
#include "controller/whole_body_controller/force_task.hpp"
#include "controller/whole_body_controller/ihwbc/ihwbc.hpp"
#include "controller/whole_body_controller/internal_constraint.hpp"
#include "controller/whole_body_controller/task.hpp"
#include "planner/locomotion/dcm_planner/dcm_planner.hpp"

// task managers
#include "controller/crab_controller/crab_rolling_joint_constraint.hpp"
#include "controller/crab_controller/crab_state_provider.hpp"
#include "controller/whole_body_controller/managers/dcm_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/reaction_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/upper_body_trajectory_manager.hpp"

static double err_tol = 1e-3;

class IHWBCTest : public ::testing::Test {
protected:
  IHWBCTest() {
    cfg = YAML::LoadFile(THIS_COM "config/crab/sim/pybullet/ihwbc/pnc.yaml");

    robot = new PinocchioRobotSystem(
        THIS_COM "robot_model/crab/crab.urdf",
        THIS_COM "robot_model/crab", false, false);
    tci_container = new CrabTCIContainer(robot, cfg);

    init_com_pos.setZero();
    target_com_pos.setZero();
    init_dcm_vel.setZero();
    desired_force.setZero();
    init_torso_quat.setIdentity();
    target_torso_quat.setIdentity();
    nominal_joint_pose.resize(crab::n_adof);
    nominal_joint_pose.setZero();  // Set nominal pose for crab
    sp = CrabStateProvider::GetStateProvider();
  }

  ~IHWBCTest() {
    delete robot;
    delete tci_container;
    delete upper_body_tm;
    delete floating_base_tm;
    delete lf_SE3_tm;
    delete rf_SE3_tm;
    delete lf_max_normal_force_tm;
    delete rf_max_normal_force_tm;
    delete rolling_joint_constraint;
  }

  void SetUp() override {
    // outputs of WBC
    joint_trq_cmd = Eigen::VectorXd::Zero(crab::n_adof);
    wbc_qddot_cmd = Eigen::VectorXd::Zero(crab::n_qdot);

    // set virtual & actuated selection matrix
    std::vector<bool> act_list;
    act_list.resize(crab::n_qdot, true);
    for (int i(0); i < robot->NumFloatDof(); ++i)
      act_list[i] = false;

    int num_qdot(act_list.size());
    int num_float(robot->NumFloatDof());
    int num_active(std::count(act_list.begin(), act_list.end(), true));
    int num_passive(num_qdot - num_active - num_float);

    sa = Eigen::MatrixXd::Zero(num_active, num_qdot);
    sf = Eigen::MatrixXd::Zero(num_float, num_qdot);
    sv = Eigen::MatrixXd::Zero(num_passive, num_qdot);

    int j(0), k(0), e(0);
    for (int i(0); i < act_list.size(); ++i) {
      if (act_list[i]) {
        sa(j, i) = 1.;
        ++j;
      } else {
        if (i < num_float) {
          sf(k, i) = 1.;
          ++k;
        } else {
          sv(e, i) = 1.;
          ++e;
        }
      }
    }
  }

  void updateRobotConfiguration() {
    Eigen::Vector3d bjoint_pos(0.0, 0.0, 0.5);  // Adjust for crab's default height
    Eigen::Quaterniond bjoint_quat(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector3d bjoint_lin_vel(0.0, 0.0, 0.0);
    Eigen::Vector3d bjoint_ang_vel(0.0, 0.0, 0.0);

    Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(crab::n_adof);
    Eigen::VectorXd joint_vel = Eigen::VectorXd::Zero(crab::n_adof);

    robot->UpdateRobotModel(bjoint_pos, bjoint_quat.normalized(),
                           bjoint_lin_vel, bjoint_ang_vel, joint_pos,
                           joint_vel, true);

    init_com_pos = robot->GetRobotComPos();
    target_com_pos = init_com_pos;
    init_torso_quat = bjoint_quat;
    target_torso_quat = bjoint_quat;

    desired_force << 0.0, 0.0, 0.0, 0.0, 0.0, robot->GetTotalMass() * 9.81 / 2.0;
  }

  // ... Rest of the test class implementation follows similar pattern
  // Reference the original test file for additional methods needed
};

// Add your test cases here
TEST_F(IHWBCTest, TestCrabIHWBC) {
  // Implement your test cases
}