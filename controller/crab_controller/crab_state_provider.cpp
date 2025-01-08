#include "controller/crab_controller/crab_state_provider.hpp"
#include "controller/crab_controller/crab_definition.hpp"
#include "util/util.hpp"

CrabStateProvider *CrabStateProvider::GetStateProvider() {
  static CrabStateProvider state_provider;
  return &state_provider;
}

CrabStateProvider::CrabStateProvider() {
  util::PrettyConstructor(1, "CrabStateProvider");

  servo_dt_ = 0.00125;
  data_save_freq_ = 1;

  count_ = 0;
  current_time_ = 0.;

  stance_foot_ = crab_link::back_left__foot_link;
  prev_stance_foot_ = crab_link::back_left__foot_link;

  rot_world_local_ = Eigen::Matrix3d::Identity();

  b_lf_contact_ = true;
  b_rf_contact_ = true;
  b_request_change_swing_leg_ = false;
  b_swing_leg_ = end_effector::LFoot;

  com_vel_est_.setZero();

  state_ = 1;      // crab_states::kInitialize or crab_states::wbic::kInitialize
  prev_state_ = 1; // crab_states::kInitialize or crab_states::wbic::kInitialize

  planning_id_ = 0;

  floating_base_jidx_ = {0, 1, 2, 3, 4, 5};

  rot_world_local_ = Eigen::Matrix3d::Identity();
}
