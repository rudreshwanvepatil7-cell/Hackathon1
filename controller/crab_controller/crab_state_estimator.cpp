#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "controller/filter/digital_filters.hpp"

#include "controller/crab_controller/crab_definition.hpp"
#include "controller/crab_controller/crab_interface.hpp"
#include "controller/crab_controller/crab_state_estimator.hpp"
#include "controller/crab_controller/crab_state_provider.hpp"

#if B_USE_ZMQ
#include "controller/draco_controller/crab_data_manager.hpp"
#endif

#include <string>

CrabStateEstimator::CrabStateEstimator(PinocchioRobotSystem *robot,
                                       const YAML::Node &cfg)
    : StateEstimator(robot), R_imu_base_com_(Eigen::Matrix3d::Identity()),
      global_leg_odometry_(Eigen::Vector3d::Zero()),
      prev_base_joint_pos_(Eigen::Vector3d::Zero()), b_first_visit_(true),
      com_vel_exp_filter_(nullptr) {
  util::PrettyConstructor(1, "CrabStateEstimator");

  sp_ = CrabStateProvider::GetStateProvider();

  // assume start with double support
  sp_->b_lf_contact_ = true;
  sp_->b_rf_contact_ = true;

  // TODO change according to actual IMU position and orientation
  R_imu_base_com_ = robot_->GetLinkIsometry(crab_link::base_link).linear();

  try {
    // set the foot that will be used to estimate base in first_visit
    int foot_frame = util::ReadParameter<int>(cfg["state_estimator"],
                                              "foot_reference_frame");
    if (foot_frame == 0) {
      sp_->stance_foot_ = crab_link::back_left__foot_link;
      sp_->prev_stance_foot_ = crab_link::back_left__foot_link;
    } else {
      sp_->stance_foot_ = crab_link::back_right__foot_link;
      sp_->prev_stance_foot_ = crab_link::back_right__foot_link;
    }

    com_vel_filter_type_ =
        util::ReadParameter<int>(cfg["state_estimator"], "com_vel_filter_type");
    if (com_vel_filter_type_ == com_vel_filter::kMovingAverage) {
      Eigen::Vector3d num_data_com_vel = util::ReadParameter<Eigen::Vector3d>(
          cfg["state_estimator"], "num_data_com_vel");
      com_vel_mv_avg_filter_.clear();
      for (int i = 0; i < 3; i++) {
        com_vel_mv_avg_filter_.push_back(
            new SimpleMovingAverage(num_data_com_vel[i]));
      }
    } else if (com_vel_filter_type_ == com_vel_filter::kExponentialSmoother) {
      double time_constant = util::ReadParameter<double>(
          cfg["state_estimator"], "com_vel_time_constant");
      Eigen::VectorXd com_vel_err_limit = util::ReadParameter<Eigen::VectorXd>(
          cfg["state_estimator"], "com_vel_err_limit");
      com_vel_exp_filter_ = new ExponentialMovingAverageFilter(
          sp_->servo_dt_, time_constant, Eigen::VectorXd::Zero(3),
          -com_vel_err_limit, com_vel_err_limit);
    } else if (com_vel_filter_type_ == com_vel_filter::kLowPassFilter) {
      double cut_off_period =
          util::ReadParameter<double>(cfg["state_estimator"], "cut_off_period");
      com_vel_lp_filter_ =
          new LowPassVelocityFilter(sp_->servo_dt_, cut_off_period, 3);
    }
  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }

#if B_USE_MATLOGGER
  logger_ = XBot::MatLogger2::MakeLogger("/tmp/draco_state_estimator_data");
  logger_->set_buffer_mode(XBot::VariableBuffer::Mode::producer_consumer);
  appender_ = XBot::MatAppender::MakeInstance();
  appender_->add_logger(logger_);
  appender_->start_flush_thread();
#endif
}

CrabStateEstimator::~CrabStateEstimator() {
  while (!com_vel_mv_avg_filter_.empty()) {
    delete com_vel_mv_avg_filter_.back();
    com_vel_mv_avg_filter_.pop_back();
  }
  if (com_vel_exp_filter_ != nullptr)
    delete com_vel_exp_filter_;
}

void CrabStateEstimator::Initialize(CrabSensorData *sensor_data) {
  robot_->UpdateRobotModel(
      Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), sensor_data->joint_pos_,
      sensor_data->joint_vel_, false);

  // save data
#if B_USE_MATLOGGER
  if (sp_->count_ % sp_->data_save_freq_ == 0) {
    // joint encoder data
    logger_->add("joint_pos_act", sensor_data->joint_pos_);
    logger_->add("joint_vel_act", sensor_data->joint_vel_);
  }
#endif
}

void CrabStateEstimator::Update(CrabSensorData *sensor_data) {

  std::cout << "[State Estimator] Update Not Implemented Yet" << std::endl;
}

void CrabStateEstimator::UpdateGroundTruthSensorData(
    CrabSensorData *sensor_data) {
  Eigen::Vector4d base_joint_ori = sensor_data->base_joint_quat_;
  Eigen::Quaterniond base_joint_quat(base_joint_ori[3], base_joint_ori[0],
                                     base_joint_ori[1], base_joint_ori[2]);

  robot_->UpdateRobotModel(
      sensor_data->base_joint_pos_, base_joint_quat.normalized(),
      sensor_data->base_joint_lin_vel_, sensor_data->base_joint_ang_vel_,
      sensor_data->joint_pos_, sensor_data->joint_vel_, true);

#if B_USE_ZMQ
  if (sp_->count_ % sp_->data_save_freq_ == 0) {
    CrabDataManager *dm = CrabDataManager::GetDataManager();

    dm->data_->est_base_joint_pos_ = sensor_data->base_joint_pos_;
    dm->data_->est_base_joint_ori_ = sensor_data->base_joint_quat_;
    dm->data_->joint_positions_ = sensor_data->joint_pos_;

    dm->data_->b_lfoot_ = sp_->b_lf_contact_;
    dm->data_->b_rfoot_ = sp_->b_rf_contact_;
  }
#endif
#if B_USE_MATLOGGER
  if (sp_->count_ % sp_->data_save_freq_ == 0) {
    // joint encoder data
    logger_->add("joint_pos_act", sensor_data->joint_pos_);
    logger_->add("joint_vel_act", sensor_data->joint_vel_);
  }
#endif
}
