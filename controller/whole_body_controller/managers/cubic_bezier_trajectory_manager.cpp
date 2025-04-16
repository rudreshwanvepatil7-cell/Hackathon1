#include "controller/whole_body_controller/managers/cubic_bezier_trajectory_manager.hpp"
#include "util/interpolation.hpp"

/*!
 * Compute foot swing trajectory with a bezier curve
 * @param phase : How far along we are in the swing (0 to 1)
 * @param swingTime : How long the swing should take (seconds)
 */

template <typename T>
void CubicBezierTrajectoryManager<T>::ComputeSwingTrajectoryBezier(
    T phase, T swing_time) {
  p_ = CubicBezier<Eigen::Matrix<T, 3, 1>>(p0_, pf_, phase);
  v_ = CubicBezierFirstDerivative<Eigen::Matrix<T, 3, 1>>(p0_, pf_, phase) /
       swing_time;
  a_ = CubicBezierSecondDerivative<Eigen::Matrix<T, 3, 1>>(p0_, pf_, phase) /
       (swing_time * swing_time);

  _ComputeZSwingTrajectoryBezier(phase, swing_time);
}

template <typename T>
void CubicBezierTrajectoryManager<T>::ComputeSwingTrajectoryBezier(T phase) {
  ComputeSwingTrajectoryBezier(phase, T(1.0));
}

template <typename T>
void CubicBezierTrajectoryManager<T>::ComputeCurvedSwingTrajectoryBezier(
    T phase, T swing_time) {
  T theta0 = atan2(p0_[1] - com_p0_[1], p0_[0] - com_p0_[0]);
  T r = (p0_ - com_p0_).norm();
  p_ = CurvedCubicBezier(_GetComPos(phase), theta0, r, com_yaw_, phase);
  v_ = CurvedCubicBezierFirstDerivative(_GetComVel(phase), theta0, r, com_yaw_,
                                        phase) /
       swing_time;
  a_ = CurvedCubicBezierSecondDerivative(0.0, theta0, r, com_yaw_, phase) /
       (swing_time * swing_time);

  _ComputeZSwingTrajectoryBezier(phase, swing_time);
}

template <typename T>
void CubicBezierTrajectoryManager<T>::ComputeCurvedSwingTrajectoryBezier(
    T phase) {
  ComputeCurvedSwingTrajectoryBezier(phase, T(1.0));
}

template <typename T>
void CubicBezierTrajectoryManager<T>::_ComputeZSwingTrajectoryBezier(
    T phase, T swing_time) {
  T zp, zv, za;

  if (phase < T(0.5)) {
    zp = CubicBezier<T>(p0_[2], p0_[2] + height_, phase * 2);
    zv = CubicBezierFirstDerivative<T>(p0_[2], p0_[2] + height_, phase * 2) *
         2 / swing_time;
    za = CubicBezierSecondDerivative<T>(p0_[2], p0_[2] + height_, phase * 2) *
         4 / (swing_time * swing_time);
  } else {
    zp = CubicBezier<T>(p0_[2] + height_, pf_[2], phase * 2 - 1);
    zv =
        CubicBezierFirstDerivative<T>(p0_[2] + height_, pf_[2], phase * 2 - 1) *
        2 / swing_time;
    za = CubicBezierSecondDerivative<T>(p0_[2] + height_, pf_[2],
                                        phase * 2 - 1) *
         4 / (swing_time * swing_time);
  }

  p_[2] = zp;
  v_[2] = zv;
  a_[2] = za;
}

template <typename T> void CubicBezierTrajectoryManager<T>::_UpdateComVars() {
  T d = (com_pf_ - com_p0_).norm();
  T r = sqrt(d * d / (2 * (1 - cos(com_yaw_))));
  com_center_ =
      (com_p0_ + com_pf_) / 2 +
      sign(com_yaw_) * sqrt(r * r - d * d / 4) *
          Vector3<T>(com_p0_[1] - com_pf_[1], com_pf[0] - com_p0_[0]) / d;
  com_theta0_ = atan2(com_p0_[1] - center[1], com_p0[0] - center[0]);
}

template <typename T>
Vector3<T> CubicBezierTrajectoryManager<T>::_GetComPos(T phase) {
  T t = com_yaw_ * phase - com_theta0_;
  return com_center_ + Vector3<T>(cos(t), sin(t), 0.0);
}

template <typename T>
Vector3<T> CubicBezierTrajectoryManager<T>::_GetComVel(T phase) {
  T t = com_yaw_ * phase - com_theta0_;
  return com_yaw_ * Vector3<T>(-sin(t), cos(t));
}

template class CubicBezierTrajectoryManager<double>;
