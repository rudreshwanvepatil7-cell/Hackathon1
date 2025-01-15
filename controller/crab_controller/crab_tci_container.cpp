#include "controller/crab_controller/crab_tci_container.hpp"

#include <utility>

#include "controller/crab_controller/crab_definition.hpp"
#include "controller/crab_controller/crab_task/crab_com_xy_task.hpp"
#include "controller/crab_controller/crab_task/crab_com_z_task.hpp"
#include "controller/whole_body_controller/basic_contact.hpp"
#include "controller/whole_body_controller/basic_task.hpp"

CrabTCIContainer::CrabTCIContainer(PinocchioRobotSystem *robot,
                                   const YAML::Node &cfg)
    : TCIContainer(robot) {
  util::PrettyConstructor(2, "CrabTCIContainer");

  //=============================================================
  // Tasks List
  //=============================================================
  jpos_task_ = new JointTask(robot_);
  com_xy_task_ = new CrabCoMXYTask(robot_);
  com_z_task_ = new CrabCoMZTask(robot_);
  torso_ori_task_ = new LinkOriTask(robot_, crab_link::base_link);
  lf_pos_task_ = new LinkPosTask(robot_, crab_link::back_left__foot_link);
  rf_pos_task_ = new LinkPosTask(robot_, crab_link::back_right__foot_link);
  lf_ori_task_ = new LinkOriTask(robot_, crab_link::back_left__foot_link);
  rf_ori_task_ = new LinkOriTask(robot_, crab_link::back_right__foot_link);
  lh_pos_task_ = new LinkPosTask(robot_, crab_link::front_left__foot_link);
  rh_pos_task_ = new LinkPosTask(robot_, crab_link::front_right__foot_link);
  lh_ori_task_ = new LinkOriTask(robot_, crab_link::front_left__foot_link);
  rh_ori_task_ = new LinkOriTask(robot_, crab_link::front_right__foot_link);

  task_map_.clear();
  task_map_.insert(std::make_pair("joint_task", jpos_task_));
  task_map_.insert(std::make_pair("com_xy_task", com_xy_task_));
  task_map_.insert(std::make_pair("com_z_task", com_z_task_));
  task_map_.insert(std::make_pair("torso_ori_task", torso_ori_task_));
  task_map_.insert(std::make_pair("lf_pos_task", lf_pos_task_));
  task_map_.insert(std::make_pair("rf_pos_task", rf_pos_task_));
  task_map_.insert(std::make_pair("lf_ori_task", lf_ori_task_));
  task_map_.insert(std::make_pair("rf_ori_task", rf_ori_task_));
  task_map_.insert(std::make_pair("lh_pos_task", lh_pos_task_));
  task_map_.insert(std::make_pair("rh_pos_task", rh_pos_task_));
  task_map_.insert(std::make_pair("lh_ori_task", lh_ori_task_));
  task_map_.insert(std::make_pair("rh_ori_task", rh_ori_task_));

  // initialize wbc cost task list
  task_unweighted_cost_map_.clear();
  task_unweighted_cost_map_.insert(std::make_pair("joint_task", NAN));
  task_unweighted_cost_map_.insert(std::make_pair("com_xy_task", NAN));
  task_unweighted_cost_map_.insert(std::make_pair("com_z_task", NAN));
  task_unweighted_cost_map_.insert(std::make_pair("torso_ori_task", NAN));
  task_unweighted_cost_map_.insert(std::make_pair("lf_pos_task", NAN));
  task_unweighted_cost_map_.insert(std::make_pair("rf_pos_task", NAN));
  task_unweighted_cost_map_.insert(std::make_pair("lf_ori_task", NAN));
  task_unweighted_cost_map_.insert(std::make_pair("rf_ori_task", NAN));
  task_unweighted_cost_map_.insert(std::make_pair("lh_pos_task", NAN));
  task_unweighted_cost_map_.insert(std::make_pair("rh_pos_task", NAN));
  task_unweighted_cost_map_.insert(std::make_pair("lh_ori_task", NAN));
  task_unweighted_cost_map_.insert(std::make_pair("rh_ori_task", NAN));
  task_weighted_cost_map_.clear();
  task_weighted_cost_map_.insert(std::make_pair("joint_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("com_xy_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("com_z_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("torso_ori_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("lf_pos_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("rf_pos_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("lf_ori_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("rf_ori_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("lh_pos_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("rh_pos_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("lh_ori_task", NAN));
  task_weighted_cost_map_.insert(std::make_pair("rh_ori_task", NAN));

  // wbc task list for inverse kinematics
  task_vector_.clear();
  task_vector_.push_back(com_z_task_);
  task_vector_.push_back(torso_ori_task_);
  task_vector_.push_back(com_xy_task_);
  task_vector_.push_back(lf_pos_task_);
  task_vector_.push_back(rf_pos_task_);
  task_vector_.push_back(lf_ori_task_);
  task_vector_.push_back(rf_ori_task_);

  //=============================================================
  // Contacts List
  //=============================================================
  lf_contact_ =
      new SurfaceContact(robot_, crab_link::back_left__foot_link, 0.3, 0.11,
                         0.04);  // params reset later
  rf_contact_ =
      new SurfaceContact(robot_, crab_link::back_right__foot_link, 0.3, 0.11,
                         0.04);  // params reset later

  contact_map_.clear();
  //  contact_map_.insert(std::make_pair("lf_contact", lf_contact_));
  //  contact_map_.insert(std::make_pair("rf_contact", rf_contact_));

  contact_vector_.clear();
  contact_vector_.push_back(lf_contact_);
  contact_vector_.push_back(rf_contact_);

  //=============================================================
  // QP Params
  //=============================================================
  //  qp_params_ = new QPParams(6, lf_contact_->Dim() + rf_contact_->Dim());

  //=============================================================
  // Tasks, Contacts parameter initialization
  //=============================================================
  this->_InitializeParameters(cfg);
}

CrabTCIContainer::~CrabTCIContainer() {
  // task
  delete jpos_task_;
  //  delete com_xy_task_;
  //  delete com_z_task_;
  //  delete torso_ori_task_;
  delete lf_pos_task_;
  delete rf_pos_task_;
  delete lf_ori_task_;
  delete rf_ori_task_;
  delete lh_pos_task_;
  delete rh_pos_task_;
  delete lh_ori_task_;
  delete rh_ori_task_;

  // contact
  delete lf_contact_;
  delete rf_contact_;

  // QP Params
  //  delete qp_params_;
}

void CrabTCIContainer::_InitializeParameters(const YAML::Node &cfg) {
  // determine which WBC
  WBC_TYPE wbc_type;
  std::string wbc_type_string =
      util::ReadParameter<std::string>(cfg, "wbc_type");
  if (wbc_type_string == "ihwbc") {
    wbc_type = WBC_TYPE::IHWBC;
  } else if (wbc_type_string == "wbic") {
    wbc_type = WBC_TYPE::WBIC;
  }

  // task
  //  com_xy_task_->SetParameters(cfg["wbc"]["task"]["com_xy_task"], wbc_type);
  //  com_z_task_->SetParameters(cfg["wbc"]["task"]["com_z_task"], wbc_type);
  //  torso_ori_task_->SetParameters(cfg["wbc"]["task"]["torso_ori_task"],
  //                                 wbc_type);
  lf_pos_task_->SetParameters(cfg["wbc"]["task"]["foot_pos_task"], wbc_type);
  rf_pos_task_->SetParameters(cfg["wbc"]["task"]["foot_pos_task"], wbc_type);
  lf_ori_task_->SetParameters(cfg["wbc"]["task"]["foot_ori_task"], wbc_type);
  rf_ori_task_->SetParameters(cfg["wbc"]["task"]["foot_ori_task"], wbc_type);
  lh_pos_task_->SetParameters(cfg["wbc"]["task"]["hand_pos_task"], wbc_type);
  rh_pos_task_->SetParameters(cfg["wbc"]["task"]["hand_pos_task"], wbc_type);
  lh_ori_task_->SetParameters(cfg["wbc"]["task"]["hand_ori_task"], wbc_type);
  rh_ori_task_->SetParameters(cfg["wbc"]["task"]["hand_ori_task"], wbc_type);

  // contact
  lf_contact_->SetParameters(cfg["wbc"]["contact"]);
  rf_contact_->SetParameters(cfg["wbc"]["contact"]);
}
