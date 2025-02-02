#pragma once
#include "controller/state_machine.hpp"

class CrabStateProvider;
class CrabControlArchitecture;
class MinJerkCurveVec;

class Contact : public StateMachine {
 public:
  Contact(const StateId state_id, PinocchioRobotSystem *robot,
          CrabControlArchitecture *ctrl_arch);
  ~Contact();

  void FirstVisit() override;
  void OneStep() override;
  void LastVisit() override;
  bool EndOfState() override;

  StateId GetNextState() override;

  void SetParameters(const YAML::Node &node) override;

 private:
  CrabControlArchitecture *ctrl_arch_;
  CrabStateProvider *sp_;

  Eigen::VectorXd target_joint_pos_;
  Eigen::VectorXd init_joint_pos_;

  bool b_stay_here_;
  double wait_time_;

  MinJerkCurveVec *min_jerk_curves_;
};
