#pragma once
#include "controller/control_architecture.hpp"
#if B_USE_FOXGLOVE
#include "UI/foxglove/client/parameter_subscriber.hpp"
#endif
#include <any>

#include "util/util.hpp"

namespace crab_states {
constexpr int kInitialize = 1;
constexpr int kApproach = 2;
constexpr int kContact = 3;
constexpr int kReorient = 4; 
}  // namespace crab_states

class CrabController;
class CrabTCIContainer;
class FloatingBaseTrajectoryManager;
class EndEffectorTrajectoryManager;
class HandTrajectoryManager;
class CrabStateProvider;
class TaskHierarchyManager;

class CrabControlArchitecture : public ControlArchitecture {
 public:
  CrabControlArchitecture(PinocchioRobotSystem *robot, const YAML::Node &cfg);
  virtual ~CrabControlArchitecture();

  void GetCommand(void *command) override;

  CrabTCIContainer *tci_container_;

  FloatingBaseTrajectoryManager *floating_base_tm_;
  EndEffectorTrajectoryManager *lf_SE3_tm_;
  EndEffectorTrajectoryManager *rf_SE3_tm_;
  EndEffectorTrajectoryManager *lh_SE3_tm_;
  EndEffectorTrajectoryManager *rh_SE3_tm_;

  //  TaskHierarchyManager *lf_pos_hm_;
  //  TaskHierarchyManager *lf_ori_hm_;
  //  TaskHierarchyManager *rf_pos_hm_;
  //  TaskHierarchyManager *rf_ori_hm_;
  //  TaskHierarchyManager *lh_pos_hm_;
  //  TaskHierarchyManager *lh_ori_hm_;
  //  TaskHierarchyManager *rh_pos_hm_;
  //  TaskHierarchyManager *rh_ori_hm_;

 private:
  CrabController *controller_;
  CrabStateProvider *sp_;

#if B_USE_FOXGLOVE
  std::unordered_map<std::string, int *> param_map_int_;
  std::unordered_map<std::string, double *> param_map_double_;
  std::unordered_map<std::string, TaskHierarchyManager *> param_map_hm_;
  FoxgloveParameterSubscriber *param_subscriber_;
#endif
};
