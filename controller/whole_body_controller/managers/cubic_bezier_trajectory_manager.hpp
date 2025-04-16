#pragma once

#include <Eigen/Dense>

template <typename T> using Vector3 = Eigen::Matrix<T, 3, 1>;

template <typename T> class CubicBezierTrajectoryManager {
public:
  CubicBezierTrajectoryManager() {
    p0_.setZero();
    pf_.setZero();
    p_.setZero();
    v_.setZero();
    a_.setZero();
    height_ = T(0.0);

    com_p0_.setZero();
    com_pf_.setZero();
    com_center.setZero();
    com_yaw_ = T(0.0);
    com_theta0_ = T(0.0);
  }

  void ComputeSwingTrajectoryBezier(T phase, T swing_time);
  void ComputeSwingTrajectoryBezier(T phase);
  void ComputeCurvedSwingTrajectoryBezier(T phase, T swing_time);
  void ComputeCurvedSwingTrajectoryBezier(T phase);

  // setter
  void SetInitialPosition(const Vector3<T> &p0) { p0_ = p0; }
  void SetFinalPosition(const Vector3<T> &pf) { pf_ = pf; }
  void SetHeight(const T h) { height_ = h; }
  void SetComState(const Vector3<T> &p0, const Vector3<T> &pf, const T yaw) {
    com_p0_ = p0;
    com_pf_ = pf;
    com_yaw_ = yaw;

    _UpdateComVars();
  }

  // getter
  Vector3<T> GetPosition() { return p_; }
  Vector3<T> GetVelocity() { return v_; }
  Vector3<T> GetAcceleration() { return a_; }

  Vector3<T> GetFinalPosition() { return pf_; }
  Vector3<T> GetInitialPosition() { return p0_; }

private:
  Vector3<T> p0_, pf_, p_, v_, a_, com_p0_, com_pf_, com_center_;
  T height_, com_yaw_, com_theta0_;

  void _ComputeZSwingTrajectoryBezier(T phase, T swing_time);
  void _UpdateComVars();
  Vector3<T> _GetComPos(T phase);
  Vector3<T> _GetComVel(T phase);

  T sign(T n) { return (n > 0) - (n < 0); }
};
