#pragma once
#include <Eigen/Dense>
#include <vector>

class CrabStateProvider {
 public:
  static CrabStateProvider *GetStateProvider();
  ~CrabStateProvider() = default;

  // servo dt should be set outside of controller
  double servo_dt_;
  int data_save_freq_;

  int count_;
  double current_time_;

  // used in pos estimate in estimator module
  int stance_foot_;
  int prev_stance_foot_;

  bool b_lf_contact_;
  bool b_rf_contact_;
  bool b_lh_contact_;
  bool b_rh_contact_;
  bool b_request_change_swing_leg_;
  int b_swing_leg_;

  Eigen::Vector3d com_vel_est_;

  int state_;
  int prev_state_;

  bool b_use_base_height_;
  double des_com_height_;
  int planning_id_;

  std::vector<int> floating_base_jidx_;

  Eigen::Matrix3d rot_world_local_;

 private:
  CrabStateProvider();
};
