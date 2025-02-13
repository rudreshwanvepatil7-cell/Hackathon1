#pragma once
#include "controller/state_machine.hpp"
#include <Eigen/Dense>

class PinocchioRobotSystem;
class CrabControlArchitecture;
class CrabStateProvider;

class PIDController {
public:
    PIDController(double kp, double ki, double kd) 
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

    double compute(double error, double dt) {
        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        prev_error_ = error;
        return output;
    }

private:
    double kp_;
    double ki_;
    double kd_;
    double prev_error_;
    double integral_;
};

class Approach : public StateMachine {
 public:
  Approach(const StateId state_id, PinocchioRobotSystem *robot,
           CrabControlArchitecture *ctrl_arch);
  ~Approach() = default;

  void FirstVisit() override;
  void OneStep() override;
  void LastVisit() override;
  bool EndOfState() override;

  StateId GetNextState() override;

  void SetParameters(const YAML::Node &node) override;

 private:
  CrabControlArchitecture *ctrl_arch_;

  CrabStateProvider *sp_;

  // PID Controller for body orientation
  PIDController* orientation_pid_;
  
  // Previous time for dt calculation
  double prev_time_;

  // set nominal desired position/orientation (e.g., for zero acceleration cmd)
  bool b_use_fixed_foot_pos_;
  Eigen::Isometry3d nominal_lfoot_iso_;
  Eigen::Isometry3d nominal_rfoot_iso_;
  Eigen::Isometry3d nominal_lhand_iso_;
  Eigen::Isometry3d nominal_rhand_iso_;
};
