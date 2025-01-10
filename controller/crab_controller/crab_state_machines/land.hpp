#pragma once
#include "controller/state_machine.hpp"

class PinocchioRobotSystem;
class CrabControlArchitecture;
class CrabStateProvider;

class Land : public StateMachine {
public:
  Land(const StateId state_id, PinocchioRobotSystem *robot,
       CrabControlArchitecture *ctrl_arch);
  ~Land() = default;

  void FirstVisit() override;
  void OneStep() override;
  void LastVisit() override;
  bool EndOfState() override;

  StateId GetNextState() override;

  void SetParameters(const YAML::Node &node) override;

private:
  CrabControlArchitecture *ctrl_arch_;

  CrabStateProvider *sp_;

  // set nominal desired position/orientation (e.g., for zero acceleration cmd)
  bool b_use_fixed_foot_pos_;
  Eigen::Isometry3d nominal_lfoot_iso_;
  Eigen::Isometry3d nominal_rfoot_iso_;
  Eigen::Isometry3d nominal_lhand_iso_;
  Eigen::Isometry3d nominal_rhand_iso_;
};
