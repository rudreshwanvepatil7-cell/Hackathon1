#pragma once

#include "controller/whole_body_controller/task.hpp"
#include "util/util.hpp"

#if B_USE_MATLOGGER
#include <matlogger2/matlogger2.h>
#endif

class PinocchioRobotSystem;
class CrabStateProvider;
class ExponentialMovingAverageFilter;
class FirstOrderLowPassFilter;

constexpr double kGravAcc = 9.81;

namespace feedback_source {
constexpr int kCoMFeedback = 0;
} // namespace feedback_source

class CrabCoMXYTask : public Task {
public:
  CrabCoMXYTask(PinocchioRobotSystem *robot);
  ~CrabCoMXYTask();

  void UpdateOpCommand(const Eigen::Matrix3d &world_R_local =
                           Eigen::Matrix3d::Identity()) override;
  void UpdateJacobian() override;
  void UpdateJacobianDotQdot() override;

  void SetParameters(const YAML::Node &node, const WBC_TYPE wbc_type) override;

private:
  CrabStateProvider *sp_;

  int feedback_source_;

#if B_USE_MATLOGGER
  // XBot::MatLogger2::Ptr logger_;
#endif
};
