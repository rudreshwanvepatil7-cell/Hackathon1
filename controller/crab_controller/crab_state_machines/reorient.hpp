#pragma once
#include "controller/state_machine.hpp"

class CrabStateProvider;
class CrabControlArchitecture;
class MinJerkCurveVec;

class Reorient : public StateMachine {
 public:
  Reorient(const StateId state_id, PinocchioRobotSystem *robot,
           CrabControlArchitecture *ctrl_arch);
  ~Reorient();

  void FirstVisit() override;
  void OneStep() override;
  void LastVisit() override;
  bool EndOfState() override;

  StateId GetNextState() override;

  void SetParameters(const YAML::Node &node) override;

 private:
  CrabControlArchitecture *ctrl_arch_;
  CrabStateProvider *sp_;

  float duration_;
  Eigen::Vector3d target_ori_;
  Eigen::Isometry3d nominal_lfoot_iso_;
  Eigen::Isometry3d nominal_rfoot_iso_;
  Eigen::Isometry3d nominal_lhand_iso_;
  Eigen::Isometry3d nominal_rhand_iso_;

  MinJerkCurveVec *min_jerk_curves_;
};
