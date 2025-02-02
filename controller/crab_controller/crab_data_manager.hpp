#pragma once

#include <Eigen/Dense>
#include <memory>
#include <zmq.hpp>

#include "crab.pb.h"  //in build/messages/
#include "crab_definition.hpp"

struct CrabData {
 public:
  CrabData() {};
  ~CrabData() = default;

  double time_ = 0;
  int phase_ = 1;

  Eigen::Vector3d est_base_joint_pos_ = Eigen::Vector3d::Zero();
  Eigen::Vector4d est_base_joint_ori_ = Eigen::Vector4d::Zero();

  Eigen::VectorXd joint_positions_ = Eigen::VectorXd::Zero(28);

  Eigen::Vector3d des_com_pos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d act_com_pos_ = Eigen::Vector3d::Zero();

  Eigen::VectorXd lfoot_pos_ = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd rfoot_pos_ = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd lfoot_ori_ = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd rfoot_ori_ = Eigen::VectorXd::Zero(4);

  Eigen::VectorXd lfoot_rf_cmd_ = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd rfoot_rf_cmd_ = Eigen::VectorXd::Zero(6);

  Eigen::Vector2d com_xy_weight = Eigen::Vector2d::Zero();
  Eigen::Vector2d com_xy_kp = Eigen::Vector2d::Zero();
  Eigen::Vector2d com_xy_kd = Eigen::Vector2d::Zero();
  Eigen::Vector2d com_xy_ki = Eigen::Vector2d::Zero();

  double com_z_weight = 0;
  double com_z_kp = 0;
  double com_z_kd = 0;

  Eigen::Vector3d torso_ori_weight = Eigen::Vector3d::Zero();
  Eigen::Vector3d torso_ori_kp = Eigen::Vector3d::Zero();
  Eigen::Vector3d torso_ori_kd = Eigen::Vector3d::Zero();

  Eigen::Vector3d lf_pos_weight = Eigen::Vector3d::Zero();
  Eigen::Vector3d lf_pos_kp = Eigen::Vector3d::Zero();
  Eigen::Vector3d lf_pos_kd = Eigen::Vector3d::Zero();

  Eigen::Vector3d rf_pos_weight = Eigen::Vector3d::Zero();
  Eigen::Vector3d rf_pos_kp = Eigen::Vector3d::Zero();
  Eigen::Vector3d rf_pos_kd = Eigen::Vector3d::Zero();

  Eigen::Vector3d lf_ori_weight = Eigen::Vector3d::Zero();
  Eigen::Vector3d lf_ori_kp = Eigen::Vector3d::Zero();
  Eigen::Vector3d lf_ori_kd = Eigen::Vector3d::Zero();

  Eigen::Vector3d rf_ori_weight = Eigen::Vector3d::Zero();
  Eigen::Vector3d rf_ori_kp = Eigen::Vector3d::Zero();
  Eigen::Vector3d rf_ori_kd = Eigen::Vector3d::Zero();

  bool b_lfoot_ = false;
  bool b_rfoot_ = false;
  bool b_lhand_ = false;
  bool b_rhand_ = false;

  Eigen::Quaterniond quat_world_local_ = Eigen::Quaterniond::Identity();
};

// Singleton class
class CrabDataManager {
 public:
  static CrabDataManager *GetDataManager();
  ~CrabDataManager() = default;

  void InitializeSocket(const std::string &ip_address);
  void SendData();

  bool IsInitialized();  // socket initialized boolean getter

  std::unique_ptr<CrabData> data_;

 private:
  CrabDataManager();

  std::unique_ptr<zmq::context_t> context_;
  std::unique_ptr<zmq::socket_t> socket_;

  bool b_initialize_socket_;
};
