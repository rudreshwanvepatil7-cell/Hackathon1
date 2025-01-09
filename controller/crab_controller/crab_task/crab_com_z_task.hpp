#pragma once

#include "controller/whole_body_controller/task.hpp"
#include "util/util.hpp"

class CrabStateProvider;
class PinocchioRobotSystem;

namespace com_height {
constexpr int kCoM = 0;
constexpr int kBase = 1;
} // namespace com_height

class CrabCoMZTask : public Task {
public:
  CrabCoMZTask(PinocchioRobotSystem *robot);
  ~CrabCoMZTask() = default;

  void UpdateOpCommand(const Eigen::Matrix3d &world_R_local =
                           Eigen::Matrix3d::Identity()) override;
  void UpdateJacobian() override;
  void UpdateJacobianDotQdot() override;

  void SetParameters(const YAML::Node &node, const WBC_TYPE wbc_type) override;

private:
  CrabStateProvider *sp_;
  int com_height_;
};
